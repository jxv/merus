{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE KindSignatures #-}
{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE LambdaCase #-}
{-# LANGUAGE Rank2Types #-}
{-# LANGUAGE MultiWayIf #-}

module Merus where

import qualified Data.Vector.Storable as V
import qualified Data.Map as M
import SDL as SDL
import Control.Applicative
import Control.Monad
import Control.Monad.IO.Class
import Data.Function
import Data.Bits.Lens
import Control.Lens
import Linear
import Linear.Affine
import Data.StateVar
import GHC.Word
import GHC.IO
import Foreign.C.Types
import Data.Default

data Circle a = Circle {
    _circlePos :: V2 a,
    _circleRadius :: a
} deriving (Show, Eq)

makeLenses ''Circle

data Aabb a = Aabb {
    _aabbMin :: V2 a,
    _aabbMax :: V2 a
} deriving (Show, Eq)

makeLenses ''Aabb

data Shape a = CircleS (Circle a) | AabbS (Aabb a) deriving (Show, Eq)

class HasPos (p :: * -> *) where
    pos :: Num a => Lens' (p a) (V2 a)

class HasRadius (r :: * -> *) where
    radius :: Num a =>  Lens' (r a) a

class HasDimens (d :: * -> *) where
    dimens :: Num a => Lens' (d a) (V2 a)

class DirHorz d where
    _l :: (Functor f, Num a) => (a -> f a) -> d a -> f (d a)
    _r :: (Functor f, Num a) => (a -> f a) -> d a -> f (d a)

class DirVert d where
    _u :: (Functor f, Num a) => (a -> f a) -> d a -> f (d a)
    _d :: (Functor f, Num a)  => (a -> f a) -> d a -> f (d a)

class (DirHorz d, DirVert d) => Dir (d :: * -> *) where
    _lurd :: (Functor f, Num a) => (V4 a -> f (V4 a)) -> d a -> f (d a)
    _lurd = lens (\d -> V4 (d^._l) (d^._u) (d^._r) (d^._d))
                 (\dir (V4 l u r d) -> dir & (_l .~ l) . (_u .~ u) . (_r .~ r) . (_d .~ d))

instance HasPos Circle where
    pos = circlePos

instance HasRadius Circle where
    radius = circleRadius

instance HasPos Aabb where
    pos = lens _aabbMin (\Aabb{..} p -> Aabb p (_aabbMax - _aabbMin + p))

instance DirHorz Aabb where
    _l = aabbMin . _x
    _r = aabbMax . _x

instance DirVert Aabb where
    _u = aabbMin . _y
    _d = aabbMax . _y

instance Dir Aabb where
    _lurd = lens (\(Aabb (V2 l u) (V2 r d)) -> V4 l u r d)
                 (\_ (V4 l u r d) -> Aabb (V2 l u) (V2 r d))

instance HasPos Shape where
    pos = lens getter setter
     where
        getter (CircleS c) = c^.pos
        getter (AabbS a) = a^.pos
        setter (CircleS c) p = CircleS (c & pos .~ p)
        setter (AabbS a) p = AabbS (a & pos .~ p)
              

isIntersect :: Ord a => V4 a -> V4 a -> Bool
isIntersect (V4 al au ar ad) (V4 bl bu br bd) = ar >= bl && al <= br && ad >= bu && au <= bd
{-# INLINE isIntersect #-}

isInside :: Ord a => V2 a -> V4 a -> Bool
isInside (V2 x y) = isIntersect (V4 x y x y)
{-# INLINE isInside #-}

aabbVsAabb :: (Num a, Ord a) => Aabb a -> Aabb a -> Bool
aabbVsAabb a b = (a^._lurd) `isIntersect` (b^._lurd)
{-# INLINE aabbVsAabb #-}

circleVsCircle :: (Num a, Ord a) => Circle a -> Circle a -> Bool
circleVsCircle a b = let
    V2 dx dy = a^.pos - b^.pos
    rad = a^.radius + b^.radius
    in dx^2 + dy^2 < rad^2
{-# INLINE circleVsCircle #-}

clamp :: Ord a => a -> a -> a -> a
clamp low high = max low . min high
{-# INLINE clamp #-}

aabbVsCircle :: (Floating a, Ord a) => Aabb a -> Circle a -> Bool
aabbVsCircle a b = let
    inside = (b^.pos) `isInside` (a^._lurd)
    n = b^.pos - a^.pos
    w = (a^._r - a^._l)
    h = (a^._d - a^._u)
    closest = V2 (clamp 0 w (n^._x)) (clamp 0 h (n^._y))
    closest' =
        if n == closest
            then if abs (n^._x) > abs (n^._y)
                    then closest & _x .~ (if closest^._x > 0 then w else 0)
                    else closest & _y .~ (if closest^._y > 0 then h else 0)
            else closest
    normal = n - closest'
    d = dot normal normal
    r = (b^.radius) ^ 2
    in inside || d <= (b^.radius) ^ 2
{-# INLINE aabbVsCircle #-}

shapeVsShape :: (Floating a, Ord a) => Shape a -> Shape a -> Bool
shapeVsShape (AabbS a) (AabbS b) = aabbVsAabb a b
shapeVsShape (AabbS a) (CircleS c) = aabbVsCircle a c
shapeVsShape (CircleS c) (CircleS d) = circleVsCircle c d
shapeVsShape s t = shapeVsShape t s

square :: Num a => V2 a -> a -> Shape a
square p s = AabbS $ Aabb p (p + V2 s s)

rect :: Num a => V2 a -> V2 a -> Shape a
rect p d = AabbS $ Aabb p (p + d)

circle :: Num a => V2 a -> a -> Shape a
circle p r = CircleS $ Circle p r

pollEvents :: MonadIO m => m [Event]
pollEvents = do
    mevent <- pollEvent
    case mevent of
        Nothing -> return []
        Just event -> return (event:) `ap` pollEvents

data RectGraphic = RectGraphic {
    rgRect :: Rectangle Float,
    rgColor :: V4 Word8
}

data CircleGraphic = CircleGraphic {
    cgPos :: V2 Float,
    cgRadius :: Float,
    cgSegments :: Int,
    cgColor :: V4 Word8,
    cgPoints :: V.Vector (Point V2 CInt)
}

data Graphic = RectG RectGraphic | CircleG CircleGraphic

rectGraphic :: V2 Float -> V2 Float -> V4 Word8 -> RectGraphic
rectGraphic p dim color = RectGraphic (Rectangle (P p) dim) color

circleGraphic :: V2 Float -> Float -> Int -> V4 Word8 -> CircleGraphic
circleGraphic center radius segs color = let
    tau = 2 * pi
    cen = fmap round center
    angles = fmap (\i -> fromIntegral i * tau / fromIntegral (segs - 1)) [0..(segs - 1)]
    cons theta = P $ fmap round (center + V2 (cos theta * radius) (sin theta * radius))
    points = fmap cons angles
    in CircleGraphic center radius segs color (V.fromListN segs points)

toGraphic :: Shape Float -> V4 Word8 -> Graphic
toGraphic (CircleS c) = CircleG . toCircleGraphic c
toGraphic (AabbS a) = RectG . toRectGraphic a

toRectGraphic :: Aabb Float -> V4 Word8 -> RectGraphic
toRectGraphic a = rectGraphic (a^.pos) (a^.aabbMax - a^.pos)

toCircleGraphic :: Circle Float -> V4 Word8 -> CircleGraphic
toCircleGraphic c = circleGraphic (c^.pos) (c^.radius) 200

{-
translateCg :: V2 Float -> CircleGraphic -> CircleGraphic
translateCg p cg@CircleGraphic{..} = let
    isSame = fmap round p == fmap round cgPos
    cg' = circleGraphic p cgRadius cgSegments cgColor
    in if isSame then cg { cgPos = p } else cg'

sizeCg :: Float -> CircleGraphic -> CircleGraphic
sizeCg r cg@CircleGraphic{..} = let
    isSame = round r == round cgRadius
    cg' = circleGraphic cgPos r cgSegments cgColor
    in if isSame then cg { cgRadius = r } else cg'
-}

renderGraphic :: Renderer -> Graphic -> IO ()
renderGraphic ren (CircleG c) = renderCircleGraphic ren c
renderGraphic ren (RectG r) = renderRectGraphic ren r

renderCircleGraphic :: Renderer -> CircleGraphic -> IO ()
renderCircleGraphic renderer CircleGraphic{..} = do
    setRenderDrawColor renderer cgColor
    renderDrawLines renderer cgPoints

renderRectGraphic :: Renderer -> RectGraphic -> IO ()
renderRectGraphic renderer RectGraphic{..} = do
    setRenderDrawColor renderer rgColor
    renderDrawRect renderer (fmap round rgRect)

data Input = Input {
    inputExit :: Bool,
    inputLeft :: Bool,
    inputUp :: Bool,
    inputRight :: Bool,
    inputDown :: Bool
} deriving (Show, Eq)

instance Default Input where
    def = Input False False False False False

readInput :: [EventPayload] -> Input -> Input
readInput [] input = input
readInput (e:es) input = readInput es $
    if  | keyPress KeycodeEscape -> input { inputExit = True }
        | keyPress KeycodeLeft -> input { inputLeft = True }
        | keyPress KeycodeUp -> input { inputUp = True }
        | keyPress KeycodeRight -> input { inputRight = True }
        | keyPress KeycodeDown -> input { inputDown = True }
        | otherwise -> input
 where
    keyPress key = case e of
        KeyboardEvent{..}-> keysymKeycode keyboardEventKeysym == key
        _ -> False

loop :: Renderer -> Shape Float -> Shape Float -> IO ()
loop renderer c r = do
    events <- pollEvents
    let Input{..} = readInput (fmap eventPayload events) def
    let hit = shapeVsShape r c
    let redOnHit c = if hit then V4 0xff 0x00 0x00 0xff else c
    print hit
    setRenderDrawColor renderer (V4 0x00 0x00 0x00 0xff) 
    renderClear renderer
    renderGraphic renderer (toGraphic c (redOnHit $ V4 0x00 0xff 0x00 0xff))
    renderGraphic renderer (toGraphic r (redOnHit $ V4 0x00 0x00 0xff 0xff))
    renderPresent renderer
    delay 16
    let motion b = if b then 2 else 0
    let move = (V2 0 0) & (_x -~ motion inputLeft)
                        . (_y -~ motion inputUp)
                        . (_x +~ motion inputRight)
                        . (_y +~ motion inputDown)
    let c' = c & pos +~ move
    unless inputExit (loop renderer c' r)

main :: IO ()
main = do
    initialize (Just InitEverything)
    window <- createWindow "Physics" defaultWindow
    renderer <- createRenderer window (-1) defaultRenderer
    showWindow window
    screen <- getWindowSurface window
    let c = circle (V2 0 0) 50
    --let c = square (V2 0 0) 50
    let r = rect (V2 100 100) (V2 100 100)
    loop renderer c r
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit

data Clock = Clock {
    _startTime,
    _stopTime,
    _currTime :: Word32
} deriving (Show, Eq)

makeLenses ''Clock

setTime :: ASetter' Clock Word32 -> Clock -> IO Clock
setTime setter c = do
    t <- ticks
    evaluate (c & setter .~ t)

startClock, stopClock :: Clock -> IO Clock
startClock = setTime startTime
stopClock = setTime stopTime

elapsed :: Clock -> IO (Clock, Word32)
elapsed c = do
    d <- setTime currTime c
    let t = d^.currTime - c^.currTime
    evaluate (d, t)

