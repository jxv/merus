module Merus.World where

import qualified Data.Map as M
import qualified Data.Vector.Storable as VS
import qualified Data.List as L
import Control.Lens
import Linear
import Merus.Types
import Merus.Collision
import Merus.Manifold
import Data.Maybe

-- Acceleration
--    F = mA
-- => A = F * 1/m
--
-- Explicit Euler
-- x += v * dt
-- v += (1/m * F) * dt
--
-- Semi-Implicit (Symplectic) Euler
-- v += (1/m * F) * dt
-- x += v * dt

mkWorld :: World
mkWorld = World (1/60) 1 M.empty 0 (V2 0 9.8) []

integrateForces :: Float -> V2 Float -> Body a -> Body a
integrateForces dt gravity b@Body{..} = let
    vel :: V2 Float
    vel = (_bForce ^* _bInvMass + gravity) ^* (dt / 2)
    angVel = _bTorque * _bInvInertia * dt / 2
    in if _bInvMass == 0
        then b
        else b & (bVel +~ vel) . (bAngVel +~ angVel)

integrateVelocity :: Float -> V2 Float -> Body a -> Body a
integrateVelocity dt gravity b@Body{..} = let
    pos :: V2 Float
    pos = (_bVel ^* dt)
    orient = _bAngVel * dt
    b' = b & (bPos +~ pos) . (bOrient +~ orient)
    in if _bInvMass == 0 then b else integrateForces dt gravity b'
 where ppm = 50 -- pixels per meter

worldStep :: World -> World
worldStep w@World{..} = let
    list :: [(Int, Body Shape)]
    list = M.toList _wBodies
    contacts = catMaybes
        [ solveCollision aentry bentry
        | (aentry,bodies) <- zip list (tail $ L.tails list)
        , bentry <- bodies
        , (snd aentry)^.invMass /= 0 || (snd bentry)^.invMass /= 0
        ]
    -- integrate forces
    bs = M.map (integrateForces _wDeltaTime _wGravity) _wBodies
    -- initalize collision
    initStep m = manifoldInitialize _wGravity _wDeltaTime (bs M.! (m^.mfAKey)) (bs M.! (m^.mfBKey)) m
    contacts' = map initStep contacts
    -- solve collisions
    solveStep bods m = let
        (akey,bkey) = (m^.mfAKey, m^.mfBKey)
        (a,b) = manifoldApplyImpulse (bods M.! akey) (bods M.! bkey) m
        in M.insert akey a $ M.insert bkey b $ bods
    bs' = foldl solveStep bs contacts'
    -- integrate velocities
    bs'' = M.map (integrateVelocity _wDeltaTime _wGravity) bs'
    -- correct positions
    stepCorrect bods m = let
        (akey,bkey) = (m^.mfAKey, m^.mfBKey)
        (a,b) = positionalCorrection (bods M.! akey) (bods M.! bkey) m
        in M.insert akey a $ M.insert bkey b $ bods
    bs''' = foldl stepCorrect bs'' contacts'
    -- clear all forces
    bs'''' = M.map ((bForce .~ zero) . (bTorque .~ 0)) bs'''
    in w & (wBodies .~ bs'''')

worldStep'
    = worldClearForces
    . worldIntegrateVelocities
    . worldSolveCollisions
    . worldGenCollisionInfo

worldGenCollisionInfo :: World -> World
worldGenCollisionInfo w@World{..} = w & wManifolds .~ mfs
 where
    list = M.toList _wBodies
    mfs = catMaybes [
            solveCollision aentry bentry |
            (aentry,bodies) <- zip list (tail $ L.tails list),
            bentry <- bodies,
            (snd aentry)^.invMass /= 0 || (snd bentry)^.invMass /= 0
        ]

worldIntegrateForces :: World -> World 
worldIntegrateForces w = w & wBodies %~ (M.map (integrateForces (w^.wDeltaTime) (w^.wGravity)))

worldInitializeCollision :: World -> World
worldInitializeCollision w@World{..} = w & wManifolds %~ (map step)
    where step m = manifoldInitialize _wGravity _wDeltaTime (_wBodies M.! (m^.mfAKey)) (_wBodies M.! (m^.mfBKey)) m

worldSolveCollisions :: World -> World
worldSolveCollisions w = w & wBodies .~ (foldl step (w^.wBodies) (w^.wManifolds))
 where
    step bodies m = let
        (akey,bkey) = (m^.mfAKey, m^.mfBKey)
        (a,b) = manifoldApplyImpulse (bodies M.! akey) (bodies M.! bkey) m
        in M.insert akey a $ M.insert bkey b $ bodies

worldIntegrateVelocities :: World -> World
worldIntegrateVelocities w = w & wBodies %~ (M.map (integrateVelocity (w^.wDeltaTime) (w^.wGravity)))

worldCorrectPositions :: World -> World
worldCorrectPositions w = w & wBodies .~ (foldl step (w^.wBodies) (w^.wManifolds))
 where
    step bodies m = let
        (akey,bkey) = (m^.mfAKey, m^.mfBKey)
        (a,b) = positionalCorrection (bodies M.! akey) (bodies M.! bkey) m
        in M.insert akey a $ M.insert bkey b $ bodies

worldClearForces :: World -> World
worldClearForces = wBodies %~ (M.map ((bForce .~ zero) . (bTorque .~ 0)))

worldAddBody :: Body Shape -> World -> World
worldAddBody a w@World{..} = w & (wBodies .~ (M.insert _wBodyKey a _wBodies)) . (wBodyKey .~ (_wBodyKey + 1))
