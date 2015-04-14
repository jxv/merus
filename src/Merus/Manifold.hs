module Merus.Manifold where

import Control.Monad
import Control.Arrow
import Data.Maybe
import Control.Lens
import Linear
import Merus.Types
import Merus.Math
import Merus.Body
import qualified Control.Monad.State.Lazy as SL

-- Precalculations for impulse solving
manifoldInitialize :: V2 Float -> Float -> Body a -> Body b -> Manifold -> Manifold
manifoldInitialize gravity dt a b = SL.execState $ do
    mfE .= min (a^.bRestitution) (b^.bRestitution)
    mfSf .= (sqrt $ a^.bStaticFriction * b^.bStaticFriction)
    mfDf .= (sqrt $ a^.bDynamicFriction * b^.bDynamicFriction)
    let contacts x = mfContacts . x . _Just
    SL.get >>= mapMOf (contacts _1) initContact 
    SL.get >>= mapMOf (contacts _2) initContact 
 where
    initContact contact = do
        -- Calucate radii from COM to contact
        let ra = contact - a^.bPos
            rb = contact - b^.bPos
            rv = b^.bVel + xrossf' (b^.bAngVel) rb
            rv' = rv - a^.bVel - xrossf' (a^.bAngVel) ra
        when (norm rv' < norm (dt *^ gravity) + 0.0001) (mfE .= 0)
        return contact

manifoldPositionalCorrection :: Body a -> Body b -> Manifold -> (Body a, Body b)
manifoldPositionalCorrection a b Manifold{..} = let
    slop = 0.05 -- penetration allowance
    percent = 0.4 -- penetration percentage to correct
    imAB = a^.invMass + b^.invMass
    correction = ((max 0 (_mfPenetration - slop)) / imAB) *^ (_mfNormal ^* percent)
    a' = a & bPos -~ (correction ^* a^.invMass)
    b' = b & bPos +~ (correction ^* b^.invMass)
    in (a',b')

manifoldInfiniteMassCorrection :: Body a -> Body a -> (Body a, Body a)
manifoldInfiniteMassCorrection a b = join (***) (bVel .~ zero) (a,b)

manifoldApplyImpulse :: Body a -> Body a -> Manifold -> (Body a, Body a)
manifoldApplyImpulse a b mf@Manifold{..} = let
    ab' = maybe (a,b) (applyImpulse (a,b)) (fst _mfContacts)
    ab'' = maybe ab' (applyImpulse ab') (snd _mfContacts)
    in if nearZero (a^.invMass + b^.invMass)
        then manifoldInfiniteMassCorrection a b
        else ab''
 where
    applyImpulse :: (Body a, Body a) -> V2 Float -> (Body a, Body a)
    applyImpulse (a,b) contact = let
        -- calculate radii from COM to contact
        ra, rb, rv :: V2 Float
        ra = contact - a^.bPos
        rb = contact - b^.bPos
        -- relative velocity
        rv = b^.bVel + xrossf' (b^.bAngVel) rb - a^.bVel - xrossf' (a^.bAngVel) ra
        -- relative velocity along the normal
        contactVel :: Float
        contactVel = dot rv _mfNormal
        raCrossN = xrossv ra _mfNormal
        rbCrossN = xrossv rb _mfNormal
        invMassSum = a^.invMass + b^.invMass + (raCrossN^2)  * a^.invInertia + (rbCrossN^2) * b^.invInertia
        -- calculate impulse scalar
        j = (-(1 + _mfE) * contactVel) / invMassSum / count
        impulse = _mfNormal ^* j
        a' = bodyApplyImpulse (-impulse) ra a
        b' = bodyApplyImpulse impulse rb b
        -- friction impulse
        rv' = b'^.bVel + xrossf' (b'^.bAngVel) rb - a'^.bVel - xrossf' (a'^.bAngVel) rb
        t = normalize $ rv' - _mfNormal ^* dot rv' _mfNormal
        -- j tangent magnitude
        jt = (negate $ dot rv' t) / invMassSum / count
        -- coulumb's law
        tagentImpulse = if abs jt < j * _mfSf then t ^* jt else t ^* ((-j) * _mfDf)
        a'' = bodyApplyImpulse (-tagentImpulse) ra a'
        b'' = bodyApplyImpulse tagentImpulse rb b'
        -- dont' resolve if velocities are separating
        -- don't apply friction impulses
        in if | contactVel > 0 -> (a,b)
              | nearZero jt -> (a',b')
              | otherwise -> (a'',b'')
    hasContact x = if (isJust . x) _mfContacts then 1 else 0
    count = hasContact fst + hasContact snd

dualManifold :: Manifold -> Manifold
dualManifold mf = mf & mfNormal %~ negate
