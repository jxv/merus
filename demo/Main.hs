module Main where

import qualified Data.Foldable as F
import qualified Data.Vector.Storable as V
import qualified Data.Map as M
import SDL as SDL
import Control.Applicative
import Control.Monad
import Control.Monad.IO.Class
import Control.Monad.State
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
import Data.Maybe (isJust)
import Merus

data Input = Input {
    inputExit :: Bool,
    inputLeft :: Bool,
    inputUp :: Bool,
    inputRight :: Bool,
    inputDown :: Bool
} deriving (Show, Eq)

instance Default Input where
    def = Input False False False False False

data RectGraphic = RectGraphic {
    rgRect :: Rectangle Float,
    rgColor :: V4 Word8
} deriving (Show)

data CircleGraphic = CircleGraphic {
    cgPos :: V2 Float,
    cgRadius :: Float,
    cgSegments :: Int,
    cgColor :: V4 Word8,
    cgPoints :: V.Vector (Point V2 CInt)
} deriving (Show)

data Graphic
    = RectG RectGraphic
    | CircleG CircleGraphic
    deriving (Show)

data Demo = Demo {
    dRenderer :: Renderer,
    dC :: Shape Float,
    dR :: Shape Float,
    dM :: Maybe (Manifold Float)
} deriving (Show)

pollEvents :: MonadIO m => m [Event]
pollEvents = do
    mevent <- pollEvent
    case mevent of
        Nothing -> return []
        Just event -> return (event:) `ap` pollEvents

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
toGraphic (BbS a) = RectG . toRectGraphic a

toRectGraphic :: Bb Float -> V4 Word8 -> RectGraphic
toRectGraphic a = rectGraphic (a^.bbMin) (a^.bbMax - a^.bbMin) 

toCircleGraphic :: Circle Float -> V4 Word8 -> CircleGraphic
toCircleGraphic c = circleGraphic (c^.pos) (c^.radius) 200

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
renderDemo Demo{..} = do
    setRenderDrawColor dRenderer (V4 0x00 0x00 0x00 0xff) 
    renderClear dRenderer
    renderGraphic dRenderer $ toGraphic dC (redOnHit $ V4 0x00 0xff 0x00 0xff)
    renderGraphic dRenderer $ toGraphic dR (redOnHit $ V4 0x00 0x00 0xff 0xff)
    renderPresent dRenderer
 where
    redOnHit color = if isJust dM then V4 0xff 0x00 0x00 0xff else color

loop :: Demo -> IO ()
loop demo@Demo{..} = do
    events <- pollEvents
    let Input{..} = readInput (fmap eventPayload events) def
        motion b = if b then 8 else 0
        move = (V2 0 0) & (_x -~ motion inputLeft)
                        . (_y -~ motion inputUp)
                        . (_x +~ motion inputRight)
                        . (_y +~ motion inputDown)
        c' = dC & pos +~ move
        r' = dR
        mmanifold = shapeVsShape r' c'
        demo' = demo { dM = mmanifold, dC = c', dR = r' }
    F.mapM_ print mmanifold
    renderDemo demo'
    delay 16
    unless inputExit (loop demo')

main :: IO ()
main = do
    initialize (Just InitEverything)
    window <- createWindow "Physics" defaultWindow{ windowSize = V2 1200 700 }
    renderer <- createRenderer window (-1) defaultRenderer
    showWindow window
    screen <- getWindowSurface window
    let demo = Demo {
        dRenderer = renderer,
        dC = circle (V2 100 100) 50,
        dR = rect (V2 300 350) (V2 100 100),
        dM = Nothing
    }
    loop demo
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit

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

