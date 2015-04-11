module Merus.Body where

import Control.Lens
import Linear
import Merus.Types
import Merus.Math

bodySetStatic :: Body a -> Body a
bodySetStatic = (inertia .~ 0) . (mass .~ 0)

bodySetOrient :: Float -> Body Shape -> Body Shape
bodySetOrient orient = (bOrient .~ orient) -- . (setShapeOrient orient)

bodyApplyForce :: V2 Float -> Body a -> Body a
bodyApplyForce force = bForce +~ force

bodyApplyImpulse :: V2 Float -> V2 Float -> Body a -> Body a
bodyApplyImpulse impulse contact body = body
    & (bVel +~ (impulse ^* body^.bInvMass))
    . (bAngVel +~ ((body^.bInvInertia) * spin))
 where spin = xrossv contact impulse
