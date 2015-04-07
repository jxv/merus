module Merus.Types where

import qualified Data.Vector as V
import Control.Lens
import Data.Time
import Linear
import Linear.Affine
import Linear.Epsilon

-- data

data Scene = Scene {
    _sIterations :: Int,
    _sDt :: Float,
    _sObjects :: [Object],
    _sContacts :: [Manifold]
} deriving Show

data Manifold = Manifold {
    _mfPenetration :: Float, -- Depth of penetration from collison
    _mfNormal :: V2 Float, -- From a to b
    _mfContacts :: (Maybe (V2 Float), Maybe (V2 Float)), -- Points of contact during collision
    _mfE :: Float, -- Mixed restitution
    _mfDynamicFriction :: Float, -- Mixed dynamic friction
    _mfStaticFriction :: Float -- Mixed static friction
} deriving Show

data Body = Body {
    _bPos :: V2 Float,
    _bVel :: V2 Float,
    _bAngVel :: Float,
    _bTorque :: Float,
    _bOrient :: Float,
    _bForce :: V2 Float,
    _bInertia :: Float,
    _bInvInertia :: Float,
    _bMass :: Float,
    _bInvMass :: Float,
    _bStaticFriction :: Float,
    _bDynamicFriction :: Float,
    _bRestitution :: Float
} deriving Show

data Shape
    = ShapeCircle Circle
    | ShapePolygon Polygon
    deriving Show

data Polygon = Polygon {
    _pU :: M22 Float,
    _pVertCount :: Int,
    _pVerts :: V.Vector (V2 Float),
    _pNorms :: V.Vector (V2 Float)
} deriving Show

data Circle = Circle {
    _cRadius :: Float
} deriving Show

data Object = Object {
    _oBody :: Body,
    _oShape :: Shape
} deriving Show

data PolygonObject = PolygonObject {
    _poBody :: Body,
    _poPolygon :: Polygon
} deriving Show

data CircleObject = CircleObject {
    _coBody :: Body,
    _coCircle :: Circle
} deriving Show

data Clock = Clock {
    _clkStart :: UTCTime,
    _clkStop :: UTCTime,
    _clkCurr :: UTCTime
} deriving Show

-- mk lens

makeClassy ''Scene
makeClassy ''Manifold
makeClassy ''Body
makeClassy ''Shape
makeClassy ''Circle
makeClassy ''Polygon
makeClassy ''Object
makeClassy ''PolygonObject
makeClassy ''CircleObject
makeClassy ''Clock

-- classes

class ToObject a where
    toObject :: a -> Object

-- instances

instance HasBody Object where
    body = oBody

instance HasShape Object where
    shape = oShape

instance HasBody PolygonObject where
    body = poBody

instance HasPolygon PolygonObject where
    polygon = poPolygon

instance HasBody CircleObject where
    body = coBody

instance HasCircle CircleObject where
    circle = coCircle
