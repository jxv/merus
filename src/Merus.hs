{-# LANGUAGE MultiParamTypeClasses, FunctionalDependencies, FlexibleInstances #-}
module Merus where

import Control.Applicative
import Control.Lens
import Linear
import Linear.Affine

data Circle = Circle {
    _cRadius :: Float
} deriving (Show, Eq)

data Rect = Rect {
    _rRadii :: V2 Float
} deriving (Show, Eq)

data Aabb = Aabb {
    _aMin :: V2 Float,
    _aMax :: V2 Float
} deriving (Show, Eq)

data Shape
    = ShapeCircle Circle
    | ShapeRect Rect
    deriving (Show, Eq)

data Manifold = Manifold {
    _mfNormal :: V2 Float,
    _mfPenetration :: Float
} deriving (Show, Eq)

data Body a = Body {
    _bPos :: V2 Float,
    _bVel :: V2 Float,
    _bMass :: Float,
    _bRestitution :: Float,
    _bShape :: a
}


--

makeClassy ''Circle
makeClassy ''Rect
makeClassy ''Aabb
makeClassy ''Shape
makeClassy ''Manifold
makeClassy ''Body

--

class Dir a where
    _l :: Lens' a Float
    _r :: Lens' a Float
    _u :: Lens' a Float
    _d :: Lens' a Float
    _lurd :: Lens' a (V4 Float)
    _lurd = lens (\d -> V4 (d^._l) (d^._u) (d^._r) (d^._d))
                 (\dir (V4 l u r d) -> dir & (_l .~ l) . (_u .~ u) . (_r .~ r) . (_d .~ d))

class ToAabb a where
    toAabb :: a -> Aabb

class ToShape a where
    toShape :: a -> Shape

--

instance Functor Body where
    fmap f b = b { _bShape = f (_bShape b) }

instance HasCircle (Body Circle) where
    circle = bShape

instance HasRect (Body Rect) where
    rect = bShape

instance HasAabb (Body Aabb) where
    aabb = bShape

instance ToAabb (Body Circle) where
    toAabb a = let r = pure (a^.cRadius) in Aabb (a^.bPos - r) (a^.bPos + r)

instance ToAabb (Body Aabb) where
    toAabb = _bShape

instance ToAabb (Body Rect) where
    toAabb a = Aabb (a^.bPos - a^.rRadii) (a^.bPos + a^.rRadii)

instance Show a => Show (Body a) where
    show Body{..} =
        "Body {" ++
        "_bPos = " ++ show _bPos ++
        ", _bVel = " ++ show _bVel ++
        ", _bMass = " ++ show _bMass ++
        ", _bRestitution = " ++ show _bRestitution ++
        ", _bShape = " ++ show _bShape ++
        "}"

instance Eq a => Eq (Body a) where
    (==) a b =
        _bPos a == _bPos b &&
        _bVel a == _bVel b &&
        _bMass a == _bMass b &&
        _bRestitution a == _bRestitution b &&
        _bShape a == _bShape b

instance ToShape Shape where
    toShape = id

instance ToShape Rect where
    toShape = ShapeRect

instance ToShape Circle where
    toShape = ShapeCircle

instance Dir Aabb where
    _l = aMin . _x
    _r = aMax . _x
    _u = aMin . _y
    _d = aMax . _y

--

aabbPos :: Aabb -> V2 Float
aabbPos a = a^.aMin + ((a^.aMax - a^.aMin) / 2)

isIntersect :: V4 Float -> V4 Float -> Bool
isIntersect (V4 al au ar ad) (V4 bl bu br bd) = ar >= bl && al <= br && ad >= bu && au <= bd
{-# INLINE isIntersect #-}

isInside :: V2 Float -> V4 Float -> Bool
isInside (V2 x y) = isIntersect (V4 x y x y)
{-# INLINE isInside #-}

{-
aabbVsAabb :: Aabb -> Aabb -> Maybe Manifold
aabbVsAabb a b = let
    n = b^.pos - a^.pos
    xOverlap = a^.radii^._x + b^.radii^._x - abs (n^._x)
    yOverlap = a^.radii^._y + b^.radii^._y - abs (n^._y)
    in if xOverlap <= 0
        then Nothing
        else Just $
            if yOverlap > 0
            then Manifold (if n^._x < 0 then V2 (-1) 0 else zero) xOverlap 
            else Manifold (if n^._y < 0 then V2 0 (-1) else V2 0 1) yOverlap
{-# INLINE aabbVsAabb #-}

-}

rectToRect :: Body Rect -> Body Rect -> Maybe Manifold
rectToRect a b = let
    n = b^.bPos - a^.bPos
    xOverlap = a^.rRadii^._x + b^.rRadii^._x - abs (n^._x)
    yOverlap = a^.rRadii^._y + b^.rRadii^._y - abs (n^._y)
    in if xOverlap <= 0
        then Nothing
        else Just $
            if yOverlap > 0
            then Manifold (if n^._x < 0 then V2 (-1) 0 else zero) xOverlap 
            else Manifold (if n^._y < 0 then V2 0 (-1) else V2 0 1) yOverlap
{-# INLINE rectToRect #-}

{-
testAabbVsAabb :: Aabb -> Aabb -> Bool
testAabbVsAabb a b = (a^._lurd) `isIntersect` (b^._lurd)
{-# INLINE testAabbVsAabb #-}

testCircleVsCircle :: Circle -> Circle -> Bool
testCircleVsCircle a b = let
    n = a^.pos - b^.pos
    rad2 = (a^.radius + b^.radius) ^ (2 :: Int)
    in dot n n < rad2
{-# INLINE testCircleVsCircle #-}


circleVsCircle :: Circle -> Circle -> Maybe Manifold
circleVsCircle a b = let
    n = a^.pos - b^.pos
    d = norm n
    rad2 = (a^.radius + b^.radius) ^ (2 :: Int)
    in if dot n n < rad2
        then Nothing
        else Just $
            if d /= 0
            then Manifold (V2 (n^._x / d) (n^._y / d)) (rad2 - d)
            else Manifold (V2 1 0) (a^.radius)
{-# INLINE circleVsCircle #-}
-}

circleToCircle :: Body Circle -> Body Circle -> Maybe Manifold
circleToCircle a b = let
    n = a^.bPos - b^.bPos
    d = norm n
    rad2 = (a^.cRadius + b^.cRadius) ^ (2 :: Int)
    in if dot n n < rad2
        then Nothing
        else Just $
            if d /= 0
            then Manifold (V2 (n^._x / d) (n^._y / d)) (rad2 - d)
            else Manifold (V2 1 0) (a^.cRadius)
{-# INLINE circleToCircle #-}

clamp :: Ord a => a -> a -> a -> a
clamp low high = max low . min high
{-# INLINE clamp #-}

{-
testAabbVsCircle :: Aabb -> Circle -> Bool
testAabbVsCircle a b = let
    inside = (b^.pos) `isInside` (a^._lurd)
    n = b^.pos - a^.pos
    closest = V2 (clamp (-a^.radii^._x) (a^.radii^._x) (n^._x))
                 (clamp (-a^.radii^._y) (a^.radii^._y) (n^._y))
    normal = n - closest
    d = dot normal normal
    rad2 = (b^.radius) ^ 2
    in inside || d <= rad2
{-# INLINE testAabbVsCircle #-}

aabbVsCircle :: Aabb -> Circle -> Maybe Manifold
aabbVsCircle a b = let
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
{-# INLINE aabbVsCircle #-}
-}

rectToCircle :: Body Rect -> Body Circle -> Maybe Manifold
rectToCircle a b = let
    inside = (b^.bPos) `isInside` ((toAabb a)^._lurd)
    n = b^.bPos - a^.bPos
    closest = V2 (clamp (-a^.rRadii^._x) (a^.rRadii^._x) (n^._x))
                 (clamp (-a^.rRadii^._y) (a^.rRadii^._y) (n^._y))
    closest' =
        if n == closest
        then if abs (n^._x) > abs (n^._y)
            then closest & _x .~ (if closest^._x > 0 then (a^.rRadii^._x) else (-a^.rRadii^._x))
            else closest & _y .~ (if closest^._y > 0 then (a^.rRadii^._y) else (-a^.rRadii^._y))
        else closest
    normal = n - closest'
    d = dot normal normal
    r = b^.cRadius
    in if not inside && d > r ^ 2
        then Nothing
        else Just $ Manifold (if inside then -n else n) (r + d)
{-# INLINE rectToCircle #-}

{-
testShapeVsShape :: Shape -> Shape -> Bool
testShapeVsShape (ShapeAabb a) (ShapeAabb b) = testAabbVsAabb a b
testShapeVsShape (ShapeAabb a) (ShapeCircle c) = testAabbVsCircle a c
testShapeVsShape (ShapeCircle c) (ShapeCircle d) = testCircleVsCircle c d
testShapeVsShape s t = testShapeVsShape t s

shapeVsShape :: Shape -> Shape -> Maybe Manifold
shapeVsShape (ShapeAabb a) (ShapeAabb b) = aabbVsAabb a b
shapeVsShape (ShapeAabb a) (ShapeCircle c) = aabbVsCircle a c
shapeVsShape (ShapeCircle c) (ShapeCircle d) = circleVsCircle c d
shapeVsShape s t = shapeVsShape t s
-}

bodyToBody :: (ToShape a, ToShape b) => Body a -> Body b -> Maybe Manifold
bodyToBody a b = b2b (fmap toShape a) (fmap toShape b)
 where
    b2b a@Body{_bShape = ShapeRect ar}   b@Body{_bShape = ShapeRect br}   = rectToRect (ar <$ a) (br <$ b)
    b2b a@Body{_bShape = ShapeRect ar}   b@Body{_bShape = ShapeCircle bc} = rectToCircle (ar <$ a) (bc <$ b)
    b2b a@Body{_bShape = ShapeCircle ac} b@Body{_bShape = ShapeCircle bc} = circleToCircle (ac <$ a) (bc <$ b)
    b2b a                                b                                = b2b b a

resolveCollision :: Manifold -> Body a -> Body a -> (Body a, Body a)
resolveCollision m a b = let
    rv = (_bVel a) - (_bVel b)
    velTanNorm = dot rv (_mfNormal m)
    e = min (_bRestitution a) (_bRestitution b)
    j = -(1 + e) * velTanNorm
    j' = j / (recip (_bMass a) + recip (_bMass b))
    impulse = pure j' * (_mfNormal m)
    a' = a & bVel -~ (impulse / pure (a^.bMass))
    b' = b & bVel -~ (impulse / pure (a^.bMass))
    in if velTanNorm > 0 then (a,b) else (a',b')

mkSquare :: V2 Float -> Float -> Body Shape
mkSquare p s = fmap toShape $ Body p zero 0 0 (Rect $ pure s)

mkRect :: V2 Float -> V2 Float -> Body Shape
mkRect p d = fmap toShape $ Body p zero 0 0 (Rect d)

mkCircle :: V2 Float -> Float -> Body Shape
mkCircle p r = fmap toShape $ Body p zero 0 0 (Circle r)
