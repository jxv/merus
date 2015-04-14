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
    demoSteps :: Int,
    demoRenderer :: Renderer,
    demoBodies :: [Body Shape],
    demoManifold :: [Maybe Manifold],
    demoGravity :: V2 Float,
    demoDeltaTime :: Float
} deriving (Show)

instance Default Input where
    def = Input False False False False False

pollEvents :: MonadIO m => m [Event]
pollEvents = do
    mevent <- pollEvent
    case mevent of
        Nothing -> return []
        Just event -> return (event:) `ap` pollEvents

polyGraphic :: V.Vector (V2 Float) -> V4 Word8 -> ColorPoly
polyGraphic verts color = ColorPoly (V.map (P . fmap round) $ verts) color

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
renderDemo Demo{..} = do
    setRenderDrawColor demoRenderer (V4 0x00 0x00 0x00 0xff) 
    renderClear demoRenderer
    renderGraphic demoRenderer $ toGraphic (head demoBodies) (V4 0xff 0xff 0xff 0xff)
    forM_ (zip demoManifold $ tail demoBodies) $ \(mf, b) -> 
        renderGraphic demoRenderer $ toGraphic b (onHit mf $ V4 0x00 0x00 0xff 0xff)
    renderPresent demoRenderer
 where
    onHit mf color = if isJust mf then V4 0x00 0xff 0x00 0xff else color

loop :: Demo -> IO ()
loop demo@Demo{..} = do
    startTick <- ticks
    events <- pollEvents
    let Input{..} = readInput (fmap eventPayload events) def
    let motion = 10
    let orZero b v n = n + if b then v else 0
    let move v = v
            & (_x %~ (orZero inputLeft $ -motion))
            . (_y %~ (orZero inputUp $ -motion))
            . (_x %~ (orZero inputRight motion))
            . (_y %~ (orZero inputDown motion))
    let c = (head demoBodies) & (bVel %~ move)
    let manifolds = map (\s -> bodyToBody s c) (tail demoBodies)
    let demo' = demo { demoManifold = manifolds, demoBodies = c : tail demoBodies }
    let demo'' = makeDemoSteps demo'
    unless (manifolds == demoManifold) $ print manifolds
    renderDemo demo''
    endTick <- ticks
    delay (delayTime 16 startTick endTick)
    unless inputExit (loop demo'')

delayTime :: Word32 -> Word32 -> Word32 -> Word32
delayTime goal start end = if frame >= goal then 0 else goal - frame
    where frame = end - start

makeDemoSteps :: Demo -> Demo
makeDemoSteps d = iterate stepDemo d !! demoSteps d

stepDemo :: Demo -> Demo
stepDemo d@Demo{..} = let
    dt = demoDeltaTime / fromIntegral demoSteps
    bodies = map (integrateVelocity dt demoGravity) demoBodies
    in d { demoBodies = bodies }

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
    let attr :: Body Shape -> Body Shape
        attr = (mass .~ 1) . (bRestitution .~ 0.8)
    let demo = Demo {
                demoDeltaTime = 1 / 60,
                demoSteps = 10,
                demoRenderer = renderer,
                demoGravity = zero, -- V2 0 9.8,
                demoBodies = [
                    mkCircle 50 & attr . (bPos .~ V2 100 100),
                    mkSquare 10 & attr . (bPos .~ V2 300 350),
                    mkRect (V2 300 10) & attr . (bPos .~ V2 175 200)
                ],
                demoManifold = replicate (length (demoBodies demo) - 1) Nothing
            }
    loop demo
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit
