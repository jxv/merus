module Merus where

import qualified Data.Foldable as F
import Control.Applicative
import Control.Lens
import Data.Time
import Data.Time.Clock.POSIX
import GHC.Conc.IO (threadDelay)
import Linear
import Linear.Affine
import Linear.Epsilon
import Data.Maybe

data Scene = Scene {
    s_iterations :: Int,
    s_dt :: Float,
--    s_bodies :: [Body],
    s_objects :: [Object],
    s_contacts :: [Manifold]
} deriving Show

data Manifold = Manifold {
    mf_penetration :: Float, -- depth of penetration from collison
    mf_normal :: V2 Float, -- from a to b
    mf_contacts :: (Maybe (V2 Float), Maybe (V2 Float)), -- points of contact during collision
    mf_e :: Float, -- mixed retitution
    mf_df :: Float, -- mixed dynamic friction
    mf_sf :: Float -- mixed static friction
} deriving Show

data Body = Body {
    b_pos :: V2 Float,
    b_vel :: V2 Float,
    b_angVel :: Float,
    b_torque :: Float,
    b_orient :: Float, -- radians
    b_force :: V2 Float,
    b_i :: Float, -- inertia
    b_iI :: Float, -- inverse inertia
    b_m :: Float, -- mass
    b_im :: Float, --inverse mass
    b_staticFriction :: Float,
    b_dynamicFriction :: Float,
    b_restitution :: Float
} deriving Show

data Shape
    = ShapeCircle Circle
    | ShapePoly Poly
    deriving Show

data Circle = Circle {
    c_radius :: Float
} deriving Show

data Poly = Poly {
    p_u :: M22 Float,
    p_vertCount :: Int,
    p_verts :: [V2 Float],
    p_norms :: [V2 Float]
} deriving Show

data Object = Object {
    o_body :: Body,
    o_shape :: Shape
} deriving Show

data Clock = Clock {
    clk_start :: UTCTime,
    clk_stop :: UTCTime,
    clk_curr :: UTCTime
} deriving Show

-- type classes

class HasBody a where
    body :: Lens' a Body

-- iemath

clamp low high = max low . min high

biasGreaterThan a b = let
    biasRelative = 0.95
    biasAbsolute = 0.01
    in a >= b * biasRelative + a * biasAbsolute

gravityScale = 5.0

gravity = V2 0 (10 * gravityScale)

dt = 1.0 / 60.0

-- clock

startClock :: Clock -> IO Clock
startClock c = do
    t <- getCurrentTime
    return $ c { clk_start = t }

stopClock :: Clock -> IO Clock
stopClock c = do
    t <- getCurrentTime
    return $ c { clk_stop = t }

elapsedClock :: Clock  -> IO (Clock, NominalDiffTime)
elapsedClock c@Clock{..} = do
    t <- getCurrentTime
    let c' = c { clk_curr = t }
    return $ (c', t `diffUTCTime` clk_curr)

differenceClock :: Clock -> NominalDiffTime
differenceClock Clock{..} = clk_stop `diffUTCTime` clk_start

currentClock :: Clock -> POSIXTime
currentClock = utcTimeToPOSIXSeconds . clk_curr

{-
delayTime :: NominalDiffTime -> IO ()
delayTime d = threadDelay $ ceiling $ d * 1e6

frameDuration :: Integer -> DiffTime
frameDuration fps = picosecondsToDiffTime $ fromInteger (div (10 ^ (12 :: Int)) fps)

delayTill :: UTCTime -> IO ()
delayTill w = do
  n <- getCurrentTime
  delayTime $ w `diffUTCTime` n
-}

-- render

renderString :: Int -> Int -> String -> IO ()
renderString _ _ _ = return ()

-- object

computeMass :: Object -> Float -> Object
computeMass Object{..} = case o_shape of
    (ShapeCircle c) -> computeCircleMass o_body c
    (ShapePoly p) -> computePolyMass o_body p 

initializeObject :: Object -> Object
initializeObject = flip computeMass 1

-- body

mkBody :: Int -> Int -> Body
mkBody x y = Body {
    b_i = 0,
    b_iI = 0,
    b_m = 0,
    b_im = 0,
    b_pos = V2 (fromIntegral x) (fromIntegral y),
    b_vel = zero,
    b_angVel = 0,
    b_torque = 0,
    b_orient = 0,
    b_force = zero,
    b_staticFriction = 0.5,
    b_dynamicFriction = 0.3,
    b_restitution = 0.2
}

bodyApplyForce :: Body -> V2 Float -> Body
bodyApplyForce b@Body{..} f = b { b_force = b_force + f }

bodyApplyImpulse :: Body -> V2 Float -> V2 Float -> Body
bodyApplyImpulse b@Body{..} impulse contactVector = let
    vel = impulse ^* b_im
    angVel = b_iI * (xross contactVector impulse)
    in b { b_vel = b_vel + vel, b_angVel = b_angVel + angVel }
 where
    xross x y = x^._x * y^._y - x^._y * y^._x

bodySetStatic :: Body -> Body
bodySetStatic b = b { b_i = 0, b_iI = 0, b_m = 0, b_im = 0 }

objectSetOrient :: Object -> Float -> Object
objectSetOrient o@Object{..} radians = o {
    o_body = o_body { b_orient = radians },
    o_shape = shapeSetOrient o_shape radians
}

-- shape

shapeSetOrient :: Shape -> Float -> Shape
shapeSetOrient (ShapePoly p) orient = let
    s = sin orient
    c = cos orient
    in ShapePoly $ p { p_u = V2 (V2 c (-s)) (V2 s c) }
shapeSetOrient s _ = s

computeCircleMass :: Body -> Circle -> Float -> Object
computeCircleMass b@Body{..} c@Circle{..} density = let
    r2 = c_radius^2
    m = pi * r2 * density
    im = if m /= 0 then recip m else 0
    i = m * r2 
    iI = if i /= 0 then recip i else 0
    b' = b { b_m = m, b_im = im, b_i = i, b_iI = iI }
    in Object b' (ShapeCircle c)

computePolyMass :: Body -> Poly -> Float -> Object
computePolyMass b@Body{..} p@Poly{..} density = let
    kInv3 = recip 3
    iter (area, i, c) i1 = let
        p1 = p_verts !! i1
        i2 = if i1 + 1 < p_vertCount then i1 + 1 else 0
        p2 = p_verts !! i2
        d = xross p1 p2
        triArea = 0.5 * d
        area' = area + triArea
        c' = (p1 + p2) ^* (triArea * kInv3)
        intx2 = (p1^._x)^2 + p1^._x * p2^._x + (p2^._x)^2
        inty2 = (p1^._y)^2 + p1^._y * p2^._y + (p2^._y)^2
        i' = 0.25 * kInv3 * d * (intx2 + inty2)
        in (area', i', c')
    (area, i, c) = F.foldl' iter (0, 0, zero) [0..(p_vertCount - 1)]
    c' = c ^/ area
    verts = fmap (subtract c') p_verts
    --
    m = density * area
    im = if m /= 0 then recip m else 0
    i' = density * i 
    iI = if i /= 0 then recip i else 0
    b' = b { b_m = m, b_im = im, b_i = i', b_iI = iI }
    in Object b' (ShapePoly p{ p_verts = verts })
 where
    xross x y = x^._x * y^._y - x^._y * y^._x

polySetBox :: Poly -> Float -> Float -> Poly
polySetBox p hw hh = let
    verts = [
            V2 (-hw) (-hh),
            V2 hw (-hh),
            V2 hw hh,
            V2 (-hw) hh
        ]
    norms = [
            V2 0 (-1),
            V2 1 0,
            V2 0 1,
            V2 (-1) 0
        ]
    in p { p_vertCount = 4, p_verts = verts, p_norms = norms }


polySet :: Poly -> [V2 Float] -> Poly
polySet p@Poly{..} verts = undefined

-- collision

circleVsCircle :: (Floating a, Ord a) => (Body, Circle) -> (Body, Circle) -> Maybe Manifold
circleVsCircle (b1, c1) (b2, c2) = let
    normal = (b_pos b2) - (b_pos b1)
    distFromBothPos = norm normal
    rad = c_radius c1 + c_radius c2
    rad2 = rad ^ 2
    in if dot normal normal < rad2 -- Check for contact
        then Nothing
        else Just $ if distFromBothPos == 0
            then let
                penetration = c_radius c1
                normal' = V2 1 0
                contacts = (Just $ b_pos b1, Nothing)
                in Manifold {
                        mf_penetration = penetration, mf_normal = normal', mf_contacts = contacts,
                        mf_e = 0, mf_df = 0, mf_sf = 0
                    }
            else let
                penetration = rad - distFromBothPos
                normal' = normal ^/ distFromBothPos
                contacts = (Just $ normal' ^* (c_radius c1) * (b_pos b1), Nothing)
                in Manifold {
                        mf_penetration = penetration, mf_normal = normal', mf_contacts = contacts,
                        mf_e = 0, mf_df = 0, mf_sf = 0
                    }

findIncidentFace :: (V2 Float, V2 Float) -> Poly -> (Poly, Body) -> Int -> (V2 Float, V2 Float)
findIncidentFace (v1, v2) refPoly (incPoly, incBody) refIdx = let
    refNormal = p_norms refPoly !! refIdx
    refNormal' = (p_u refPoly) !* refNormal
    refNormal'' = (transpose $ p_u incPoly) !* refNormal'
    iter (minDot, incidentFace) (idx, normal) = let
        d = dot refNormal'' normal
        in if d < minDot
            then (d, idx)
            else (minDot, incidentFace)
    (_, incidentFace) = F.foldl' iter (1 / 0, 0) (zip [0..] $ p_norms incPoly)
    v1' = (p_u incPoly) !* ((p_verts incPoly) !! incidentFace) + (b_pos incBody)
    incidentFace' = if incidentFace + 1 >= p_vertCount incPoly
        then 0
        else incidentFace + 1
    v2' = (p_u incPoly) !* ((p_verts incPoly) !! incidentFace') + (b_pos incBody)
    in (v1', v2')

clip :: V2 Float -> Float -> V2 Float -> (Maybe (V2 Float), Maybe (V2 Float))
clip n c face@(V2 fx fy) = let
    -- retrieve dists from each endpoint to the line
    d1 = (dot n (pure fx)) - c
    d2 = (dot n (pure fy)) - c
    x = if d1 <= 0 then (Just . pure) fx else Nothing
    y = if d2 <= 0 then (Just . pure) fy else Nothing
    z = if d1 * d2 < 0
        then let
            alpha = d1 / (d1 - d2)
            in pure . pure $ fx + alpha * (fy - fx)
        else Nothing
    face' = if isJust x then (x, y <|> z) else (y, z)
    in face'

-- manifold
-- scene

mkScene :: Float -> Int -> Scene
mkScene dt iterations = Scene iterations dt [] []

integrateForces :: Body -> Float -> Body
integrateForces b@Body{..} dt = let
    vel = (b_force ^* b_im + gravity) ^* (dt / 2)
    angVel = b_torque * b_iI * dt / 2
    in if b_im == 0
        then b
        else b { b_vel = b_vel + vel, b_angVel = b_angVel + angVel }

integrateVelocity :: Body -> Float -> Body
integrateVelocity b@Body{..} dt = let
    pos = b_vel ^* dt
    orient = b_angVel * dt
    b' = b { b_pos = b_pos + pos, b_orient = b_orient + orient }
    in if b_im == 0
        then b
        else integrateForces b' dt

-- TODO
sceneStep :: Scene -> Scene
sceneStep s@Scene{..} = let
    s' = s { s_contacts = [] }
    in s
