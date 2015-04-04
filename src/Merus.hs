module Merus where

import Control.Applicative
import Control.Lens
import Linear
import Linear.Affine

data Circle a = Circle {
    _circlePos :: V2 a,
    _circleRadius :: a
} deriving (Show, Eq)

makeLenses ''Circle

data Bb a = Bb {
    _bbMin :: V2 a,
    _bbMax :: V2 a
} deriving (Show, Eq)

makeLenses ''Bb

data Shape a
    = CircleS (Circle a)
    | BbS (Bb a)
    deriving (Show, Eq)

data Manifold a = Manifold {
    _mfNormal :: V2 a,
    _mfPenetration :: a
} deriving (Show, Eq)

makeLenses ''Manifold

class HasPos (p :: * -> *) where
    pos :: Floating a => Lens' (p a) (V2 a)

class HasRadius (r :: * -> *) where
    radius :: Floating a =>  Lens' (r a) a

class HasRadii (r :: * -> *) where
    radii :: Floating a => Lens' (r a) (V2 a)

class HasDimens (d :: * -> *) where
    dimens :: Floating a => Lens' (d a) (V2 a)

class Dir (d :: * -> *) where
    _l :: (Functor f, Floating a) => (a -> f a) -> d a -> f (d a)
    _r :: (Functor f, Floating a) => (a -> f a) -> d a -> f (d a)
    _u :: (Functor f, Floating a) => (a -> f a) -> d a -> f (d a)
    _d :: (Functor f, Floating a)  => (a -> f a) -> d a -> f (d a)
    _lurd :: (Functor f, Floating a) => (V4 a -> f (V4 a)) -> d a -> f (d a)
    _lurd = lens (\d -> V4 (d^._l) (d^._u) (d^._r) (d^._d))
                 (\dir (V4 l u r d) -> dir & (_l .~ l) . (_u .~ u) . (_r .~ r) . (_d .~ d))

class ToBb t where
    toBb :: Floating a => t -> Bb a

instance HasPos Circle where
    pos = circlePos

instance HasRadius Circle where
    radius = circleRadius

instance HasPos Bb where
    pos = lens getter setter
     where
        getter a = a^.bbMin + a^.radii
        setter a p = let r = a^.radii in Bb (p - r) (p + r)

instance HasRadii Bb where
    radii = lens getter setter
     where
        getter a = (a^.bbMax - a^.bbMin) / 2
        setter a r = let p = a^.pos in Bb (p - r) (p + r)

instance Dir Bb where
    _l = bbMin . _x
    _r = bbMax . _x
    _u = bbMin . _y
    _d = bbMax . _y

instance HasPos Shape where
    pos = lens getter setter
     where
        getter (CircleS c) = c^.pos
        getter (BbS a) = a^.pos
        setter (CircleS c) p = CircleS (c & pos .~ p)
        setter (BbS a) p = BbS (a & pos .~ p)

isIntersect :: Ord a => V4 a -> V4 a -> Bool
isIntersect (V4 al au ar ad) (V4 bl bu br bd) = ar >= bl && al <= br && ad >= bu && au <= bd
{-# INLINE isIntersect #-}

isInside :: Ord a => V2 a -> V4 a -> Bool
isInside (V2 x y) = isIntersect (V4 x y x y)
{-# INLINE isInside #-}

bbVsBb :: (Floating a, Ord a) => Bb a -> Bb a -> Maybe (Manifold a)
bbVsBb a b = let
    n = b^.pos - a^.pos
    xOverlap = a^.radii^._x + b^.radii^._x - abs (n^._x)
    yOverlap = a^.radii^._y + b^.radii^._y - abs (n^._y)
    in if xOverlap <= 0
        then Nothing
        else Just $ if yOverlap > 0
                then Manifold (if n^._x < 0 then V2 (-1) 0 else V2 0 0) xOverlap 
                else Manifold (if n^._y < 0 then V2 0 (-1) else V2 0 1) yOverlap
{-# INLINE bbVsBb #-}

testBbVsBb :: (Floating a, Ord a) => Bb a -> Bb a -> Bool
testBbVsBb a b = (a^._lurd) `isIntersect` (b^._lurd)
{-# INLINE testBbVsBb #-}

testCircleVsCircle :: (Floating a, Ord a) => Circle a -> Circle a -> Bool
testCircleVsCircle a b = let
    n = a^.pos - b^.pos
    rad2 = (a^.radius + b^.radius) ^ (2 :: Int)
    in dot n n < rad2
{-# INLINE testCircleVsCircle #-}

circleVsCircle :: (Floating a, Ord a) => Circle a -> Circle a -> Maybe (Manifold a)
circleVsCircle a b = let
    n = a^.pos - b^.pos
    d = norm n
    rad2 = (a^.radius + b^.radius) ^ (2 :: Int)
    in if dot n n < rad2
        then Nothing
        else Just $ if d /= 0
            then Manifold (V2 (n^._x / d) (n^._y / d)) (rad2 - d)
            else Manifold (V2 1 0) (a^.radius)
{-# INLINE circleVsCircle #-}

clamp :: Ord a => a -> a -> a -> a
clamp low high = max low . min high
{-# INLINE clamp #-}

testBbVsCircle :: (Floating a, Ord a) => Bb a -> Circle a -> Bool
testBbVsCircle a b = let
    inside = (b^.pos) `isInside` (a^._lurd)
    n = b^.pos - a^.pos
    V2 xRadius yRadius = a^.radii
    closest = V2 (clamp (-xRadius) xRadius (n^._x)) (clamp (-yRadius) yRadius (n^._y))
    normal = n - closest
    d = dot normal normal
    rad2 = (b^.radius) ^ 2
    in inside || d <= rad2
{-# INLINE testBbVsCircle #-}

bbVsCircle :: (Floating a, Ord a) => Bb a -> Circle a -> Maybe (Manifold a)
bbVsCircle a b = let
    inside = (b^.pos) `isInside` (a^._lurd)
    n = b^.pos - a^.pos
    V2 xRadius yRadius = a^.radii
    closest = V2 (clamp (-xRadius) xRadius (n^._x)) (clamp (-yRadius) yRadius (n^._y))
    closest' =
        if n == closest
            then if abs (n^._x) > abs (n^._y)
                    then closest & _x .~ (if closest^._x > 0 then xRadius else (-xRadius))
                    else closest & _y .~ (if closest^._y > 0 then yRadius else (-yRadius))
            else closest
    normal = n - closest'
    d = dot normal normal
    r = b^.radius
    in if not inside && d > r^2
        then Nothing
        else Just $ Manifold (if inside then -n else n) (r + d)
{-# INLINE bbVsCircle #-}

testShapeVsShape :: (Floating a, Ord a) => Shape a -> Shape a -> Bool
testShapeVsShape (BbS a) (BbS b) = testBbVsBb a b
testShapeVsShape (BbS a) (CircleS c) = testBbVsCircle a c
testShapeVsShape (CircleS c) (CircleS d) = testCircleVsCircle c d
testShapeVsShape s t = testShapeVsShape t s

shapeVsShape :: (Floating a, Ord a) => Shape a -> Shape a -> Maybe (Manifold a)
shapeVsShape (BbS a) (BbS b) = bbVsBb a b
shapeVsShape (BbS a) (CircleS c) = bbVsCircle a c
shapeVsShape (CircleS c) (CircleS d) = circleVsCircle c d
shapeVsShape s t = shapeVsShape t s

data Body a = Body {
    bdVel :: V2 a,
    bdMass :: a,
    bdRestitution :: a
} deriving (Show, Eq)

resolveCollision :: (Floating a, Ord a) => Manifold a -> Body a -> Body a -> (Body a, Body a)
resolveCollision m a b = let
    rv = (bdVel a) - (bdVel b)
    velTanNorm = dot rv (_mfNormal m)
    e = min (bdRestitution a) (bdRestitution b)
    j = -(1 + e) * velTanNorm
    j' = j / (recip (bdMass a) + recip (bdMass b))
    impulse = pure j' * (_mfNormal m)
    a' = a { bdVel = bdVel a - impulse / pure (bdMass a) }
    b' = b { bdVel = bdVel b + impulse / pure (bdMass b) }
    in if velTanNorm > 0 then (a,b) else (a',b')

square :: Floating a => V2 a -> a -> Shape a
square p s = BbS $ Bb p (p + pure s)

rect :: Floating a => V2 a -> V2 a -> Shape a
rect p d = BbS $ Bb p (p + d)

circle :: Floating a => V2 a -> a -> Shape a
circle p r = CircleS $ Circle p r

