module Merus where

import qualified Data.Foldable as F
import qualified Data.Vector as V
import qualified Control.Monad.State.Lazy as SL
import Control.Applicative
import Control.Monad
import Control.Lens
import Data.Time
import Data.Time.Clock.POSIX
import GHC.Conc.IO (threadDelay)
import Linear
import Linear.Affine
import Linear.Epsilon
import Data.Maybe

import Merus.Types

--

instance ToObject CircleObject where
    toObject CircleObject{..} = Object { _oBody = _coBody, _oShape = ShapeCircle _coCircle }

instance ToObject PolygonObject where
    toObject PolygonObject{..} = Object { _oBody = _poBody, _oShape = ShapePolygon _poPolygon }

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
startClock = mapMOf clkStart (const getCurrentTime)

stopClock :: Clock -> IO Clock
stopClock = mapMOf clkStop (const getCurrentTime)

elapsedClock :: Clock  -> IO (Clock, NominalDiffTime)
elapsedClock c@Clock{..} = do
    t <- getCurrentTime
    let c' = c & clkCurr .~ t
    return $ (c', t `diffUTCTime` _clkCurr)

differenceClock :: Clock -> NominalDiffTime
differenceClock Clock{..} = _clkStop `diffUTCTime` _clkStart

currentClock :: Clock -> POSIXTime
currentClock = utcTimeToPOSIXSeconds . _clkCurr

-- render

renderString :: Int -> Int -> String -> IO ()
renderString _ _ _ = return ()

-- object

computeMass :: Object -> Float -> Object
computeMass Object{..} = case _oShape of
    (ShapeCircle c) -> computeCircleMass $ CircleObject _oBody c
    (ShapePolygon p) -> computePolygonMass $ PolygonObject _oBody p 

initializeObject :: Object -> Object
initializeObject = flip computeMass 1

-- body

mkBody :: Int -> Int -> Body
mkBody x y = Body {
    _bInertia = 0,
    _bInvInertia = 0,
    _bMass = 0,
    _bInvMass = 0,
    _bPos = V2 (fromIntegral x) (fromIntegral y),
    _bVel = zero,
    _bAngVel = 0,
    _bTorque = 0,
    _bOrient = 0,
    _bForce = zero,
    _bStaticFriction = 0.5,
    _bDynamicFriction = 0.3,
    _bRestitution = 0.2
}

bodyApplyForce :: Body -> V2 Float -> Body
bodyApplyForce b f = b & bForce +~ f

bodyApplyImpulse :: Body -> V2 Float -> V2 Float -> Body
bodyApplyImpulse b@Body{..} impulse contactVector = let
    vel = impulse ^* _bInvMass
    angVel = _bInvInertia * (xross contactVector impulse)
    in b & (bVel +~ vel) . (bAngVel +~ angVel)

xross :: V2 Float -> V2 Float -> Float
xross x y = x^._x * y^._y - x^._y * y^._x

bodySetStatic :: Body -> Body
bodySetStatic b = b & (bInertia .~ 0) . (bInvInertia .~ 0) . (bMass .~ 0) . (bInvMass .~ 0)

objectSetOrient :: Object -> Float -> Object
objectSetOrient o radians = o & (bOrient .~ radians) . (oShape %~ flip shapeSetOrient radians)

-- shape

shapeSetOrient :: Shape -> Float -> Shape
shapeSetOrient (ShapePolygon p) orient = let
    s = sin orient
    c = cos orient
    in ShapePolygon $ p & pU .~ (V2 (V2 c (-s)) (V2 s c))
shapeSetOrient s _ = s

computeCircleMass :: CircleObject -> Float -> Object
computeCircleMass co density = let
    r2 = (co^.cRadius) ^ 2
    m = pi * r2 * density
    im = if m /= 0 then recip m else 0
    i = m * r2 
    iI = if i /= 0 then recip i else 0
    in toObject $ co & (bMass .~ m) . (bInvMass .~ im) . (bInertia .~ i) . (bInvInertia .~ iI)

computePolygonMass :: PolygonObject -> Float -> Object
computePolygonMass po density = let
    kInv3 = recip 3
    iter (area, i, c) i1 = let
        p1 = (po^.pVerts) V.! i1
        i2 = if i1 + 1 < (po^.pVertCount) then i1 + 1 else 0
        p2 = (po^.pVerts) V.! i2
        d = xross p1 p2
        triArea = 0.5 * d
        area' = area + triArea
        c' = (p1 + p2) ^* (triArea * kInv3)
        intx2 = (p1^._x)^2 + p1^._x * p2^._x + (p2^._x)^2
        inty2 = (p1^._y)^2 + p1^._y * p2^._y + (p2^._y)^2
        i' = 0.25 * kInv3 * d * (intx2 + inty2)
        in (area', i', c')
    (area, i, c) = F.foldl' iter (0, 0, zero) [0..(po^.pVertCount - 1)]
    c' = c ^/ area
    verts = fmap (subtract c') (po^.pVerts)
    --
    m = density * area
    im = if m /= 0 then recip m else 0
    i' = density * i 
    iI = if i /= 0 then recip i else 0
    po' = po & (bMass .~ m) . (bInvMass .~ im) . (bInertia .~ i') . (bInvInertia .~ iI) . (pVerts .~ verts)
    in toObject po'

polygonSetBox :: Polygon -> Float -> Float -> Polygon
polygonSetBox p hw hh = let
    verts = V.fromListN 4 [
            V2 (-hw) (-hh),
            V2 hw (-hh),
            V2 hw hh,
            V2 (-hw) hh
        ]
    norms = V.fromListN 4 [
            V2 0 (-1),
            V2 1 0,
            V2 0 1,
            V2 (-1) 0
        ]
    in p & (pVertCount .~ 4) . (pVerts .~ verts) . (pNorms .~ norms)

polygonSet :: Polygon -> [V2 Float] -> Polygon
polygonSet p verts = undefined
   
polygonGetSupport :: Polygon -> V2 Float -> V2 Float
polygonGetSupport Polygon{..} dir = let
    best = (zero, 1 / 0)
    iter (bestVert, bestProj) idx = let
        v = _pVerts V.! idx
        proj = dot v dir
        in if proj > bestProj
            then (v, proj)
            else (bestVert, bestProj)
    (bestVert', _) = F.foldl' iter best [0..(_pVertCount - 1)]
    in bestVert'

-- collision

dispatch :: Object -> Object -> Maybe Manifold
dispatch (Object b1 (ShapeCircle c1)) (Object b2 (ShapeCircle c2)) = let
    co1 = CircleObject b1 c1
    co2 = CircleObject b2 c2
    in circleVsCircle co1 co2
dispatch (Object b1 (ShapePolygon p1)) (Object b2 (ShapePolygon p2)) = let
    po1 = PolygonObject b1 p1
    po2 = PolygonObject b2 p2
    in polygonVsPolygon po1 po2
dispatch (Object b1 (ShapeCircle c1)) (Object b2 (ShapePolygon p2)) = undefined
dispatch a b = dispatch b a

circleVsCircle :: (Floating a, Ord a) => CircleObject -> CircleObject -> Maybe Manifold
circleVsCircle a b = let
    normal = b^.bPos - a^.bPos
    distFromBothPos = norm normal
    rad = a^.cRadius + b^.cRadius
    rad2 = rad ^ (2 :: Int)
    in if dot normal normal < rad2 -- Check for contact
        then Nothing
        else Just $ if distFromBothPos == 0
            then let
                penetration = a^.cRadius
                normal' = V2 1 0
                contacts = (Just $ a^.bPos, Nothing)
                in Manifold {
                        _mfPenetration = penetration, _mfNormal = normal', _mfContacts = contacts,
                        _mfE = 0, _mfDynamicFriction = 0, _mfStaticFriction = 0
                    }
            else let
                penetration = rad - distFromBothPos
                normal' = normal ^/ distFromBothPos
                contacts = (Just $ normal' ^* (a^.cRadius) * (a^.bPos), Nothing)
                in Manifold {
                        _mfPenetration = penetration, _mfNormal = normal', _mfContacts = contacts,
                        _mfE = 0, _mfDynamicFriction = 0, _mfStaticFriction = 0
                    }

findAxisLeastPenetration :: PolygonObject -> PolygonObject -> (Int, Float)
findAxisLeastPenetration a b = let
    iter best@(bestIdx, bestDist) idx = let
        -- Retrieve a face normal from A
        n = (a^.pNorms) V.! idx
        nw = a^.pU !* n
        -- Transform face normal into B's model space
        buT = transpose (b^.pU)
        n' = buT !* nw
        -- Retrieve support point from B along -n
        s = polygonGetSupport (b^.polygon) (-n')
        -- Retrieve vertex on face from A, transform into B's model space
        v = (a^.pVerts) V.! idx
        v' = (a^.pU) !* v + (a^.bPos)
        v'' = v' - (b^.bPos)
        v''' = buT !* v''
        -- Computer penetration distance (in B's model space)
        d = dot n' (s - v''')
        -- Store greatest distance
        best' = if d > bestDist then (idx, d) else best
        in best'
    in F.foldl' iter (0, -1/0) [0..(a^.pVertCount - 1)]

findIncidentFace :: (V2 Float, V2 Float) -> Polygon -> (Polygon, Body) -> Int -> (V2 Float, V2 Float)
findIncidentFace (v1, v2) refPolygon (incPolygon, incBody) refIdx = let
    refNormal = _pNorms refPolygon V.! refIdx
    -- Calculate normal in incident's frame of reference
    refNormal' = (_pU refPolygon) !* refNormal
    refNormal'' = (transpose $ _pU incPolygon) !* refNormal'
    -- Find most anti-normal face on incident polygongon
    iter (minDot, incidentFace) (idx, normal) = let
        d = dot refNormal'' normal
        in if d < minDot
            then (d, idx)
            else (minDot, incidentFace)
    (_, incidentFace) = F.foldl' iter (1 / 0, 0) (zip [0..] $ V.toList $ _pNorms incPolygon)
    -- Faces vertices for incidentFace
    v1' = _pU incPolygon !* (_pVerts incPolygon V.! incidentFace) + _bPos incBody
    incidentFace' = if incidentFace + 1 >= _pVertCount incPolygon
        then 0
        else incidentFace + 1
    v2' = _pU incPolygon !* (_pVerts incPolygon V.! incidentFace') + _bPos incBody
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

polygonVsPolygon :: PolygonObject -> PolygonObject -> Maybe Manifold
polygonVsPolygon a b = let
    -- Check for a separating axis with A's face planes
    -- Check for a separating axis with B's face planes
    -- Determine which shape contains reference face
    -- World space incident face
    in Nothing

-- manifold

-- Generate contact information
manifoldSolve :: Object -> Object -> Manifold -> Manifold
manifoldSolve a b = flip fromMaybe (dispatch a b)

-- Precalculations for impulse solving
manifoldInitialize :: Object -> Object -> Manifold -> Manifold
manifoldInitialize a b = SL.execState $ do
    mfE .= min (a^.bRestitution) (b^.bRestitution)
    mfStaticFriction .= (sqrt $ a^.bStaticFriction * b^.bStaticFriction)
    mfDynamicFriction .= (sqrt $ a^.bDynamicFriction * b^.bDynamicFriction)
    let contacts x = mfContacts . x . _Just
    SL.get >>= mapMOf (contacts _1) initContact 
    SL.get >>= mapMOf (contacts _2) initContact 
 where
    initContact contact = do
        -- Calucate radii from COM to contact
        let ra = contact - a^.bPos
            rb = contact - b^.bPos
            rv = b^.bVel + pure (xross (pure $ b^.bAngVel) rb)
            rv' = rv - a^.bVel - pure (xross (pure $ a^.bAngVel) ra)
        when (norm rv' < norm (dt * gravity) + 0.0001) (mfE .= 0)
        return contact

-- Solve impulse and apply
manifoldApplyImpulse :: Object -> Object -> Manifold -> Manifold
manifoldApplyImpulse a b = SL.execState $ do
    -- infinite mass correction
    let (a',b') = if nearZero (a^.bInvMass + b^.bInvMass)
            then (a & bVel .~ zero, b & bVel .~ zero)
            else (a,b)
    return ()

-- Naive correction of positional penetration
manifoldPositionalCorrection :: Manifold -> Manifold
manifoldPositionalCorrection = undefined

manifoldInfiniteMassCorrection :: Object -> Object -> (Object, Object)
manifoldInfiniteMassCorrection a b = (a & bVel .~ (V2 0 0), b & bVel .~ (V2 0 0))

-- scene

mkScene :: Float -> Int -> Scene
mkScene dt iterations = Scene iterations dt [] []

integrateForces :: Body -> Float -> Body
integrateForces b@Body{..} dt = let
    vel = (_bForce ^* _bInvMass + gravity) ^* (dt / 2)
    angVel = _bTorque * _bInvInertia * dt / 2
    in if _bInvMass == 0
        then b
        else b & (bVel +~ vel) . (bAngVel +~ angVel)

integrateVelocity :: Body -> Float -> Body
integrateVelocity b@Body{..} dt = let
    pos = _bVel ^* dt
    orient = _bAngVel * dt
    b' = b & (bPos +~ pos) . (bOrient +~ orient)
    in if _bInvMass == 0
        then b
        else integrateForces b' dt

-- TODO
sceneStep :: Scene -> Scene
sceneStep s@Scene{..} = let
    s' = s & sContacts .~ []
    in s
