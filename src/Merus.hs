module Merus where

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

aabbPos :: Aabb -> V2 Float
aabbPos a = (a^.aMax + a^.aMin) / 2

isIntersect :: V4 Float -> V4 Float -> Bool
isIntersect (V4 al au ar ad) (V4 bl bu br bd) = ar >= bl && al <= br && ad >= bu && au <= bd
{-# INLINE isIntersect #-}

isInside :: V2 Float -> V4 Float -> Bool
isInside (V2 x y) = isIntersect (V4 x y x y)
{-# INLINE isInside #-}

rectToRect :: Body Rect -> Body Rect -> Maybe Manifold
rectToRect a b = let
    n :: V2 Float
    n = b^.bPos - a^.bPos
    overlap :: V2 Float
    overlap = a^.rRadii + b^.rRadii - abs n
    in if overlap^._x <= 0
        then Nothing
        else Just $
            if overlap^._y > 0
            then mkManifold (if n^._x < 0 then V2 (-1) 0 else zero) (overlap^._x)
            else mkManifold (if n^._y < 0 then V2 0 (-1) else V2 0 1) (overlap^._y)
{-# INLINE rectToRect #-}

circleToCircle :: Body Circle -> Body Circle -> Maybe Manifold
circleToCircle a b = let
    n :: V2 Float
    n = a^.bPos - b^.bPos
    d :: Float
    d = norm n
    rad2 :: Float
    rad2 = (a^.cRadius + b^.cRadius) ^ (2 :: Int)
    in if dot n n < rad2
        then Nothing
        else Just $
            if d /= 0
            then mkManifold (V2 (n^._x / d) (n^._y / d)) (rad2 - d)
            else mkManifold (V2 1 0) (a^.cRadius)
{-# INLINE circleToCircle #-}

mkManifold :: V2 Float -> Float -> Manifold
mkManifold v n = Manifold v n 0 0  (Nothing, Nothing) 0 0 0

clamp :: Ord a => a -> a -> a -> a
clamp low high = max low . min high
{-# INLINE clamp #-}

clampF :: (Applicative f, Ord a) => f a -> f a -> f a -> f a
clampF low high n = max <$> low <*> (min <$> high <*> n)

rectToCircle :: Body Rect -> Body Circle -> Maybe Manifold
rectToCircle a b = let
    inside :: Bool
    inside = (b^.bPos) `isInside` ((toAabb a)^._lurd)
    n :: V2 Float
    n = b^.bPos - a^.bPos
    closest :: V2 Float
    closest = clampF (negate $ a^.rRadii) (a^.rRadii) n
    closest' =
        if n == closest
        then if abs (n^._x) > abs (n^._y)
            then closest & _x .~ (if closest^._x > 0 then (a^.rRadii^._x) else (-a^.rRadii^._x))
            else closest & _y .~ (if closest^._y > 0 then (a^.rRadii^._y) else (-a^.rRadii^._y))
        else closest
    normal :: V2 Float
    normal = n - closest'
    d, r :: Float
    d = dot normal normal
    r = b^.cRadius
    in if not inside && d > r ^ 2
        then Nothing
        else Just $ mkManifold (if inside then -n else n) (r + d)
{-# INLINE rectToCircle #-}

bodyToBody :: (ToShape a, ToShape b) => Body a -> Body b -> Maybe Manifold
bodyToBody a b = b2b (fmap toShape a) (fmap toShape b)
 where
    b2b :: Body Shape -> Body Shape -> Maybe Manifold
    b2b a@Body{_bShape = ShapeRect ar}   b@Body{_bShape = ShapeRect br}   = rectToRect (ar <$ a) (br <$ b)
    b2b a@Body{_bShape = ShapeRect ar}   b@Body{_bShape = ShapeCircle bc} = rectToCircle (ar <$ a) (bc <$ b)
    b2b a@Body{_bShape = ShapeCircle ac} b@Body{_bShape = ShapeCircle bc} = circleToCircle (ac <$ a) (bc <$ b)
    b2b a                                b                                = b2b b a

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

mkCircle' :: Float -> Body Circle
mkCircle' = mkBody . Circle

mkSquare :: Float -> Body Shape
mkSquare = fmap toShape . mkSquare'

mkRect :: V2 Float -> Body Shape
mkRect = fmap toShape . mkRect'

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
