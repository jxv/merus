module Main where

import qualified Data.Foldable as F
import qualified Data.Vector.Storable as V
import qualified Data.Map as M
import qualified Data.List as L
import SDL as SDL
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

{-
makeLittleStep :: FloatType -> World -> World
makeLittleStep ts world = let
    bs :: [Body]
    bs = bodies world
    contactingPairs = contacts
    contactsWithBody :: [(Body, [(Body, Maybe Contact)])]
    contactsWithBody = [
            (bs !! i, [
                    (bs !! j, mbC) 
                    | (i', j, mbC) <- contactingPairs
                    , i' == i
                ])
            | i <- [0..length bs - 1]
        ]
    modifyBody :: (Body, [(Body, Maybe Contact)]) -> Body
    modifyBody (b, contacts) =
        if isStatic b
        then b
        else foldl applyCollisionImpulse b contacts
    modifiedBodies :: [Body]
    modifiedBodies = map modifyBody contactsWithBody
    in world {
            bodies = [integrate (config world) ts b | b <- modifiedBodies]
        }
-}

stepDemo :: Demo -> Demo
stepDemo d@Demo{..} = let
    dt = demoDeltaTime / fromIntegral demoSteps
    bodies = map (integrateVelocity dt demoGravity) demoBodies
    in d { demoBodies = bodies }

contacts :: [Body Shape] -> [(Int, Int, Manifold)]
contacts bodies = concat $ [
        let bi = bodies !! i
            bj = bodies !! j
            mmanifold = bodyToBody bi bj
        in fromMaybe [] $ fmap (\mf -> [(i, j, mf), (j, i, dualManifold mf)]) mmanifold
        | i <- [0..bodiesLen - 1]
        , j <- [(i+1)..bodiesLen - 1]
    ]
 where bodiesLen = length bodies

contactsByBody :: [Body Shape] -> [(Body Shape, [(Body Shape, Manifold)])]
contactsByBody bodies = let
    mood :: [[(Int, (Int, Manifold))]]
    mood = L.groupBy (\a b -> a^._1 == b^._1) . map (\(a,b,c) -> (a,(b,c))) $ contacts bodies
    thinking = flip map mood $ \list@((i,_):_) -> (bodies !! i, map (first (bodies !!)) $ map snd list)
    in thinking 

modifyBody :: (Body Shape, [(Body Shape, Manifold)]) -> Body Shape
modifyBody (b@Body{..}, elements) = if _bType == BodyStatic then b else F.foldl' const b elements

{-
applyCollisionImpulse :: Body -> (Body, Maybe Contact) -> Body
applyCollisionImpulse b1 (_, Nothing) = b1
applyCollisionImpulse b1 (b2, Just contact)
    | isStatic b1 = b1
    | isStatic b2 = applyCollisionImpulse b1 (getPhantomBody b2, Just contact)
    | otherwise = let
    solution = uncurry gaussSolve (collisionLinearSystem b1 b2 contact)
    v = Vec (solution !! 0) (solution !! 1) (solution !! 2) 
    w = Vec (solution !! 6) (solution !! 7) (solution !! 8) 
    in setAngularVelocity w $ setLinearVelocity v b1
-}

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
                demoGravity = V2 0 9.8,
                demoBodies = [
                    mkCircle 50 & attr . (bPos .~ V2 100 100),
                    mkSquare 10 & attr . (bPos .~ V2 300 350),
                    mkRect (V2 300 10) & attr . (bPos .~ V2 350 400)
                ],
                demoManifold = replicate (length (demoBodies demo) - 1) Nothing
            }
    loop demo
    freeSurface screen
    destroyRenderer renderer
    destroyWindow window
    quit
