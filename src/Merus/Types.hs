{-# LANGUAGE MultiParamTypeClasses, FunctionalDependencies, FlexibleInstances #-}
module Merus.Types where 

import qualified Data.Vector as V
import Control.Applicative
import Control.Lens
import Control.Lens.Indexed
import Linear
import Linear.Affine
import Merus.Math

data Circle = Circle {
    _cRadius :: Float
} deriving (Show, Eq)

data Rect = Rect {
    _rRadii :: V2 Float
} deriving (Show, Eq)

data Shape
    = ShapeCircle Circle
    | ShapeRect Rect
    deriving (Show, Eq)

data Aabb = Aabb {
    _aMin :: V2 Float,
    _aMax :: V2 Float
} deriving (Show, Eq)

data Manifold = Manifold {
    _mfNormal :: V2 Float,
    _mfPenetration :: Float,
    _mfA :: Int, -- body A's index
    _mfB :: Int, -- body B's index
    --_mfContacts :: [V2 Float], -- points of collision contacts
    _mfContacts :: (Maybe (V2 Float), Maybe (V2 Float)),
    _mfE :: Float, -- mixed restitution
    _mfDf :: Float, -- mixed dynamic friction
    _mfSf :: Float -- mixed static friction
} deriving (Show, Eq)

data BodyType
    = Static
    | Dynamic
    deriving (Show, Eq)

data Body a = Body {
    _bType :: BodyType,
    _bPos :: V2 Float,
    _bVel :: V2 Float,
    _bTorque :: Float,
    _bAngVel :: Float,
    _bOrient :: Float,
    _bForce :: V2 Float,
    _bInertia :: Float,
    _bInvInertia :: Float,
    _bMass :: Float,
    _bInvMass :: Float,
    _bStaticFriction :: Float,
    _bDynamicFriction :: Float,
    _bRestitution :: Float,
    _bShape :: a
}

data World = World {
    _wDeltaTime :: Float,
    _wIterations :: Int,
    _wBodies :: V.Vector (Body Shape),
    _wManifolds :: [Manifold]
}

--

makeClassy ''Circle
makeClassy ''Rect
makeClassy ''Aabb
makeClassy ''Shape
makeClassy ''Manifold
makeClassy ''Body
makeClassy ''World

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

class HasMass a where
    massSimple :: Lens' a Float
    invMassSimple :: Lens' a Float
    mass :: Lens' a Float
    mass = interrelateInv massSimple invMassSimple
    invMass :: Lens' a Float
    invMass = interrelateInv invMassSimple massSimple

class HasInertia a where
    inertiaSimple :: Lens' a Float
    invInertiaSimple :: Lens' a Float
    inertia :: Lens' a Float
    inertia = interrelateInv inertiaSimple invInertiaSimple
    invInertia :: Lens' a Float
    invInertia = interrelateInv invInertiaSimple inertiaSimple

--

instance Functor Body where
    fmap f b = b { _bShape = f (_bShape b) }

instance HasMass (Body a) where
    massSimple = bMass
    invMassSimple = bInvMass

instance HasInertia (Body a) where
    inertiaSimple = bInertia
    invInertiaSimple = bInvInertia

instance HasCircle (Body Circle) where
    circle = bShape

instance HasRect (Body Rect) where
    rect = bShape

instance HasAabb (Body Aabb) where
    aabb = bShape

instance ToAabb (Body Circle) where
    toAabb a = let r = pure (a^.cRadius) in Aabb (a^.bPos - r) (a^.bPos + r)

instance ToAabb (Body Rect) where
    toAabb a = Aabb (a^.bPos - a^.rRadii) (a^.bPos + a^.rRadii)

instance Show a => Show (Body a) where
    show Body{..} =
        "Body {" ++
        "_bPos = " ++ show _bPos ++
        ", _bVel = " ++ show _bVel ++
        ", _bMass = " ++ show _bMass ++
        ", _bInvMass = " ++ show _bInvMass ++
        ", _bRestitution = " ++ show _bRestitution ++
        ", _bShape = " ++ show _bShape ++
        "}"

instance Eq a => Eq (Body a) where
    (==) a b =
        _bPos a == _bPos b &&
        _bVel a == _bVel b &&
        _bMass a == _bMass b &&
        _bInvMass a == _bInvMass b &&
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

interrelateInv :: Lens' a Float -> Lens' a Float -> Lens' a Float
interrelateInv a b = lens (^.a) (\x y -> x & (a .~ y) . (b .~ recipNoInf y))
