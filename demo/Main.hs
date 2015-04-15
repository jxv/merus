module Main where

import qualified Data.Foldable as F
import qualified Data.Vector.Storable as V
import qualified Data.Map as M
import qualified Data.List as L
import SDL
import Control.Applicative
import Control.Monad
import Control.Monad.IO.Class
import Control.Monad.State
import Data.Function
import Data.Bits.Lens
import Data.Traversable
import Control.Lens
import Linear
import Linear.Affine
import Data.StateVar
import GHC.Word
import GHC.IO
import Foreign.C.Types
import Data.Default
import Data.Maybe (isJust, catMaybes, fromMaybe)
import Control.Arrow (first)

import Merus
import Merus.Types
import Merus.Aabb
import Merus.Body
import Merus.Collision
import Merus.Manifold
import Merus.Shape
import Merus.World

data Input = Input {
    inputExit :: Bool,
    inputLeft :: Bool,
    inputUp :: Bool,
    inputRight :: Bool,
    inputDown :: Bool
} deriving (Show, Eq)

data ColorPoly = ColorPoly {
    cpVertices :: V.Vector (Point V2 CInt),
    cpColor :: V4 Word8
} deriving (Show)

data ColorCircle = ColorCircle {
    ccPos :: V2 Float,
    ccRadius :: Float,
    ccSegments :: Int,
    ccColor :: V4 Word8,
    ccPoints :: V.Vector (Point V2 CInt)
} deriving (Show)

data Graphic
    = GraphicPoly ColorPoly
    | GraphicCircle ColorCircle
    deriving (Show)

data Demo = Demo {
    demoRenderer :: Renderer,
    demoWorld :: World
} deriving (Show)

instance Default Input where
    def = Input False False False False False

instance HasWorld Demo where
    world = lens demoWorld (\d w -> d{ demoWorld = w })

pollEvents :: MonadIO m => m [Event]
pollEvents = do
    mevent <- pollEvent
    case mevent of
        Nothing -> return []
        Just event -> return (event:) `ap` pollEvents

scaleGraphic :: Num a => a -> a
scaleGraphic = (*) 10

polyGraphic :: V.Vector (V2 Float) -> V4 Word8 -> ColorPoly
polyGraphic verts color = ColorPoly (V.map (P . fmap round . scaleGraphic) $ verts) color

circleGraphic :: V2 Float -> Float -> Int -> V4 Word8 -> ColorCircle
circleGraphic center radius segs color = let
    tau = 2 * pi
    cen = fmap round center
    angles = fmap (\i -> fromIntegral i * tau / fromIntegral (segs - 1)) [0..(segs - 1)]
    cons theta = P $ fmap (round . scaleGraphic) (center + V2 (cos theta * radius) (sin theta * radius))
    points = fmap cons angles
    in ColorCircle center radius segs color (V.fromListN segs points)

toGraphic :: Body Shape -> V4 Word8 -> Graphic
toGraphic a = case _bShape a of
    ShapeCircle s -> GraphicCircle . toColorCircle (s <$ a)
    ShapePoly s -> GraphicPoly . toColorPoly (s <$ a)

toColorPoly :: Body Poly -> V4 Word8 -> ColorPoly
toColorPoly a c = polyGraphic (V.map (+ a^.bPos^._xy) (a^.pVertices)) c

toColorCircle :: Body Circle -> V4 Word8 -> ColorCircle
toColorCircle a = circleGraphic (a^.bPos) (a^.cRadius) 30

renderGraphic :: Renderer -> Graphic -> IO ()
renderGraphic ren (GraphicCircle c) = renderColorCircle ren c
renderGraphic ren (GraphicPoly p) = renderColorPoly ren p

renderColorCircle :: Renderer -> ColorCircle -> IO ()
renderColorCircle renderer ColorCircle{..} = do
    setRenderDrawColor renderer ccColor
    renderDrawLines renderer ccPoints

renderColorPoly :: Renderer -> ColorPoly -> IO ()
renderColorPoly ren ColorPoly{..} = do
    setRenderDrawColor ren cpColor
    renderDrawLines ren cpVertices
    let vert i = cpVertices V.! i
    renderDrawLine ren (vert $ V.length cpVertices - 1) (vert 0)

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

renderDemo :: Demo -> IO ()
renderDemo d@Demo{..} = do
    setRenderDrawColor demoRenderer (V4 0x00 0x00 0x00 0xff) 
    renderClear demoRenderer
    mapM_ (renderGraphic demoRenderer . flip toGraphic maxBound) $ d^.wBodies
    renderPresent demoRenderer

loop :: Demo -> IO ()
loop d@Demo{..} = do
    startTick <- ticks
    events <- pollEvents
    let Input{..} = readInput (fmap eventPayload events) def
    let motion = 150
    let orZero b v n = n + if b then v else 0
    let move v = v
            & (_x %~ (orZero inputLeft $ -motion))
            . (_y %~ (orZero inputUp $ -motion))
            . (_x %~ (orZero inputRight motion))
            . (_y %~ (orZero inputDown motion))
            . (_y %~ (orZero inputDown motion))
    let d' = d & (world %~ worldStep) . (wBodies %~ (M.adjust (bForce %~ move) 1))
    renderDemo d'
    endTick <- ticks
    delay (delayTime 16 startTick endTick)
    unless inputExit (loop d')

delayTime :: Word32 -> Word32 -> Word32 -> Word32
delayTime goal start end = if frame >= goal then 0 else goal - frame
    where frame = end - start

mkDemo :: Renderer -> Demo
mkDemo ren = let
    circles = [mkCircle 1 & attr . (bPos .~ V2 (fromIntegral i) 5) | i <- [4,16..64]]
    polys = []
    others = zip [2..] $ circles ++ polys
    bodies = M.fromList $ [
            (0, bodySetStatic $  mkCircle 5 & attr . (bPos .~ V2 32.5 17)),
            (1, bodySetStatic $ mkRect (V2 30 1) & attr . (bPos .~ V2 34 30))
        ] ++ others
    w = mkWorld & (wBodies .~ bodies)
    in Demo ren w
 where
    attr :: Body Shape -> Body Shape
    attr = (mass .~ 1) . (bRestitution .~ 0.2) . (bDynamicFriction .~ 0.2) . (bStaticFriction .~ 0.4)

main :: IO ()
main = do
    initialize (Just InitEverything)
    displays <- getDisplays
    let size :: V2 CInt
        size = fmap (flip div 2) (displayModeSize (head (displayModes (head displays))))
    window <- createWindow "merus demo" defaultWindow { windowSize = size }
    renderer <- createRenderer window (-1) defaultRenderer
    showWindow window
    screen <- getWindowSurface window
    let demo = mkDemo renderer
    loop demo
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit
