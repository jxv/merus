module Merus where

import qualified Data.Vector as V
import Control.Applicative
import Control.Lens
import Control.Lens.Indexed
import Linear
import Linear.Affine

import Merus.Types
import Merus.Aabb
import Merus.Body
import Merus.Collision
import Merus.Shape
import Merus.World

--

resolveCollision :: Manifold -> Body a -> Body b -> (Body a, Body b)
resolveCollision m a b = let
    rv :: V2 Float
    rv = a^.bVel - b^.bVel
    velAlongNorm :: Float
    velAlongNorm = dot rv (m^.mfNormal)
    e :: Float
    e = min (a^.bRestitution) (b^.bRestitution)
    j, j' :: Float
    j = -(1 + e) * velAlongNorm
    j' = j / (recip (a^.bMass) + recip (b^.bMass))
    impulse :: V2 Float
    impulse = pure j' * (m^.mfNormal)
    a' = a & bVel -~ (impulse / pure (a^.bMass))
    b' = b & bVel +~ (impulse / pure (b^.bMass))
    in if velAlongNorm > 0 then (a,b) else (a',b')

-- Correct floating point error
positionalCorrection :: Body a -> Body b -> Manifold -> (Body a, Body b)
positionalCorrection a b Manifold{..} = let
    percent = 0.4 -- usually 0.2 to 0.8
    slop = 0.05 -- usually 0.01 to 0.1 (to stop jitters)
    correction = (max 0 (_mfPenetration - slop) / (a^.bInvMass + b^.bInvMass)) * percent *^ _mfNormal
    a' = a & bPos -~ ((pure $ a^.bInvMass) * correction)
    b' = b & bPos +~ ((pure $ b^.bInvMass) * correction)
    in (a',b')

mkBody :: a -> Body a
mkBody = Body Dynamic nil nil nil nil nil nil nil nil nil nil nil nil nil
 where
    nil :: Num a => a
    nil = fromInteger 0

mkSquare' :: Float -> Body Rect
mkSquare' = mkBody . (Rect . pure)

mkRect' :: V2 Float -> Body Rect
mkRect' = mkBody . Rect

mkPoly' :: V.Vector (V2 Float) -> V.Vector (V2 Float) -> Body Poly
mkPoly' verts norms = mkBody $ Poly identity verts norms

mkCircle' :: Float -> Body Circle
mkCircle' = mkBody . Circle


--mkRect :: V2 Float -> Body Shape
--mkRect = fmap toShape . mkRect'

mkSquare :: Float -> Body Shape
mkSquare s = fmap (toShape . polySetBox s s) $ mkPoly' V.empty V.empty

mkPoly :: V.Vector (V2 Float) -> V.Vector (V2 Float) -> Body Shape
mkPoly verts norms = fmap toShape $ mkPoly' verts norms

mkCircle :: Float -> Body Shape
mkCircle = fmap toShape . mkCircle'

integrateForces :: Float -> V2 Float -> Body a -> Body a
integrateForces dt gravity b@Body{..} = let
    vel :: V2 Float
    vel = (_bForce ^* _bInvMass + gravity) ^* (dt / 2)
    -- angVel = _bTorque * _bInvInertia * dt / 2
    in if _bInvMass == 0
        then b
        else b & (bVel +~ vel) -- . (bAngVel +~ angVel)

integrateVelocity :: Float -> V2 Float -> Body a -> Body a
integrateVelocity dt gravity b@Body{..} = let
    pos :: V2 Float
    pos = (_bVel ^* dt)
    -- orient = _bAngVel * dt
    b' = b & (bPos +~ pos) -- . (bOrient +~ orient)
    in if _bInvMass == 0 then b else integrateForces dt gravity b'
 where ppm = 50 -- pixels per meter

dualManifold :: Manifold -> Manifold
dualManifold mf = mf & mfNormal %~ negate
