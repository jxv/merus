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


data ColorRect = ColorRect {
    crRect :: Rectangle Float,
    crColor :: V4 Word8
} deriving (Show)

data ColorCircle = ColorCircle {
    ccPos :: V2 Float,
    ccRadius :: Float,
    ccSegments :: Int,
    ccColor :: V4 Word8,
    ccPoints :: V.Vector (Point V2 CInt)
} deriving (Show)

data Graphic
    = GraphicRect ColorRect
    | GraphicCircle ColorCircle
    deriving (Show)

data Demo = Demo {
    demoRenderer :: Renderer,
    demoBodies :: [Body Shape],
    demoManifold :: Maybe Manifold
} deriving (Show)


instance Default Input where
    def = Input False False False False False

pollEvents :: MonadIO m => m [Event]
pollEvents = do
    mevent <- pollEvent
    case mevent of
        Nothing -> return []
        Just event -> return (event:) `ap` pollEvents

rectGraphic :: V2 Float -> V2 Float -> V4 Word8 -> ColorRect
rectGraphic p dim color = ColorRect (Rectangle (P p) dim) color

circleGraphic :: V2 Float -> Float -> Int -> V4 Word8 -> ColorCircle
circleGraphic center radius segs color = let
    tau = 2 * pi
    cen = fmap round center
    angles = fmap (\i -> fromIntegral i * tau / fromIntegral (segs - 1)) [0..(segs - 1)]
    cons theta = P $ fmap round (center + V2 (cos theta * radius) (sin theta * radius))
    points = fmap cons angles
    in ColorCircle center radius segs color (V.fromListN segs points)

toGraphic :: Body Shape -> V4 Word8 -> Graphic
toGraphic a = case _bShape a of
    ShapeCircle s -> GraphicCircle . toColorCircle (s <$ a)
    ShapeRect s -> GraphicRect . toColorRect (s <$ a)

toColorRect :: Body Rect -> V4 Word8 -> ColorRect
toColorRect a = let b = toAabb a in rectGraphic (b^.aMin) (b^.aMax - b^.aMin) 

toColorCircle :: Body Circle -> V4 Word8 -> ColorCircle
toColorCircle a = circleGraphic (a^.bPos) (a^.cRadius) 200

renderGraphic :: Renderer -> Graphic -> IO ()
renderGraphic ren (GraphicCircle c) = renderColorCircle ren c
renderGraphic ren (GraphicRect r) = renderColorRect ren r

renderColorCircle :: Renderer -> ColorCircle -> IO ()
renderColorCircle renderer ColorCircle{..} = do
    setRenderDrawColor renderer ccColor
    renderDrawLines renderer ccPoints

renderColorRect :: Renderer -> ColorRect -> IO ()
renderColorRect renderer ColorRect{..} = do
    setRenderDrawColor renderer crColor
    renderDrawRect renderer (fmap round crRect)

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
    setRenderDrawColor demoRenderer (V4 0x00 0x00 0x00 0xff) 
    renderClear demoRenderer
    renderGraphic demoRenderer $ toGraphic (head demoBodies) (onHit $ V4 0xff 0xff 0xff 0xff)
    forM_ (tail demoBodies) $ \s -> 
        renderGraphic demoRenderer $ toGraphic s (onHit $ V4 0x00 0x00 0xff 0xff)
    renderPresent demoRenderer
 where
    onHit color = if isJust demoManifold then V4 0x00 0xff 0x00 0xff else color

loop :: Demo -> IO ()
loop demo@Demo{..} = do
    events <- pollEvents
    let Input{..} = readInput (fmap eventPayload events) def
        motion b = if b then 8 else 0
        move = (V2 0 0) & (_x -~ motion inputLeft)
                        . (_y -~ motion inputUp)
                        . (_x +~ motion inputRight)
                        . (_y +~ motion inputDown)
        c = (head demoBodies) & bPos +~ move
        mmanifold = msum $ map (\s -> bodyToBody s c) (tail demoBodies)
        demo' = demo { demoManifold = mmanifold, demoBodies = c : tail demoBodies }
    unless (mmanifold == demoManifold) $
        F.mapM_ print mmanifold
    renderDemo demo'
    delay 16
    unless inputExit (loop demo')

main :: IO ()
main = do
    initialize (Just InitEverything)
    window <- createWindow "merus demo" defaultWindow{ windowSize = V2 1200 700 }
    renderer <- createRenderer window (-1) defaultRenderer
    showWindow window
    screen <- getWindowSurface window
    let demo = Demo {
        demoRenderer = renderer,
        demoBodies = [
            mkCircle (V2 100 100) 50,
            mkRect (V2 300 350) (V2 100 100),
            mkRect (V2 500 350) (V2 150 100)
        ],
        demoManifold = Nothing
    }
    loop demo
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit
