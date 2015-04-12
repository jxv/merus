module Merus.Collision where

import qualified Data.Vector as V
import qualified Data.Foldable as F
import Control.Applicative
import Control.Monad
import Control.Lens
import Control.Lens.Indexed
import Data.Maybe
import Linear
import Linear.Affine

import Merus.Types
import Merus.Math

polyGetSupport :: Body Poly -> V2 Float -> V2 Float
polyGetSupport a dir = let
    best = (zero, 1 / 0)
    iter (bestVert, bestProj) idx = let
        v = (a^.pVertices) V.! idx
        proj = dot v dir
        in if proj > bestProj then (v, proj) else (bestVert, bestProj)
    (bestVert', _) = F.foldl' iter best [0..(V.length (a^.pVertices) - 1)]
    in bestVert'

findAxisLeastPenetration :: Body Poly -> Body Poly -> (Int, Float)
findAxisLeastPenetration a b = let
    iter best@(bestIdx, bestDist) idx = let
        -- Retrieve a face normal from A
        n = (a^.pNormals) V.! idx
        nw = (a^.pU) !* n
        -- Transform face normal into B's model space
        buT = transpose (b^.pU)
        n' = buT !* nw
        -- Retrieve support point from B along -n
        s = polyGetSupport b (-n')
        -- Retrieve vertex on face from A, transform into B's model space
        v = (a^.pVertices) V.! idx
        v' = (a^.pU) !* v + (a^.bPos)
        v'' = v' - (b^.bPos)
        v''' = buT !* v''
        -- Computer penetration distance (in B's model space)
        d = dot n' (s - v''')
        -- Store greatest distance
        best' = if d > bestDist then (idx, d) else best
        in best'
    in F.foldl' iter (0, -1/0) [0..(V.length (a^.pVertices) - 1)]

findIncidentFace :: Body Poly -> Body Poly -> Int -> (V2 Float, V2 Float)
findIncidentFace ref inc refIdx = let
    refNormal = (ref^.pNormals) V.! refIdx
    -- Calculate normal in incident's frame of reference
    refNormal' = (ref^.pU) !* refNormal
    refNormal'' = (transpose $ inc^.pU) !* refNormal'
    -- Find most anti-normal face on incident polygongon
    iter (minDot, incidentFace) (idx, normal) = let
        d = dot refNormal'' normal
        in if d < minDot
            then (d, idx)
            else (minDot, incidentFace)
    (_, incidentFace) = F.foldl' iter (1 / 0, 0) (zip [0..] $ V.toList $ inc^.pNormals)
    -- Faces vertices for incidentFace
    v1 = (inc^.pU) !* ((inc^.pVertices) V.! incidentFace) + inc^.bPos
    incidentFace' = if incidentFace + 1 >= (V.length $ inc^.pVertices) then 0 else incidentFace + 1
    v2 = (inc^.pU) !* ((inc^.pVertices) V.! incidentFace') + inc^.bPos
    in (v1, v2)

clip :: V2 Float -> Float -> (V2 Float, V2 Float) -> (Maybe (V2 Float), Maybe (V2 Float))
clip n c (fx, fy) = let
    -- retrieve dists from each endpoint to the line
    d1, d2 :: Float
    d1 = dot n fx - c
    d2 = dot n fy - c
    -- if negative (behind plane) clip
    x = if d1 <= 0 then Just fx else Nothing
    y = if d2 <= 0 then Just fy else Nothing
    -- if the points are on different sides of the plane
    z = if d1 * d2 < 0
        then let
            alpha = d1 / (d1 - d2)
            in Just $ fx + alpha *^ (fy - fx)
        else Nothing
    face' = if isJust x then (x, y <|> z) else (y, z)
    in face'

aabbPos :: Aabb -> V2 Float
aabbPos a = (a^.aMax + a^.aMin) / 2

mkManifold :: V2 Float -> Float -> Manifold
mkManifold normal penetration = Manifold normal penetration 0 0  (Nothing, Nothing) 0 0 0

isIntersect :: V4 Float -> V4 Float -> Bool
isIntersect (V4 al au ar ad) (V4 bl bu br bd) = ar >= bl && al <= br && ad >= bu && au <= bd
{-# INLINE isIntersect #-}

isInside :: V2 Float -> V4 Float -> Bool
isInside (V2 x y) = isIntersect (V4 x y x y)
{-# INLINE isInside #-}

circleToCircle :: Body Circle -> Body Circle -> Maybe Manifold
circleToCircle a b = let
    -- translational vector is the normal
    normal = b^.bPos - a^.bPos
    distFromBothPos = norm normal
    rad = a^.cRadius + b^.cRadius
    rad2 = rad^(2 :: Int)
    in if dot normal normal < rad2
        then Nothing -- Not in contact
        else Just $ if distFromBothPos == 0
            then let
                penetration = a^.cRadius
                normal' = V2 1 0
                contacts = (Just $ a^.bPos, Nothing)
                in (mkManifold normal' penetration) & mfContacts .~ contacts
            else let
                penetration = rad - distFromBothPos
                normal' = normal ^/ distFromBothPos
                contacts = (Just $ normal' ^* (a^.cRadius) * (a^.bPos), Nothing)
                in (mkManifold normal' penetration) & mfContacts .~ contacts
{-# INLINE circleToCircle #-}
{-
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
-}

circleToPoly :: Body Circle -> Body Poly -> Maybe Manifold
circleToPoly a b = do
    -- Transform circle center to polygon model space
    let center :: V2 Float
        center = (transpose $ b^.pU) !* (a^.bPos - b^.bPos)
    -- Find edge with minimum penetration
    -- Exact concept as using support points in polygon vs polygon
    let findEdgeWithMinPen :: Int -> (Float, Int) -> V2 Float -> Maybe (Float, Int)
        findEdgeWithMinPen i (s, fn) vert = let
            s' = dot ((b^.pNormals) V.! i) (center - vert)
            in if s > a^.cRadius
                then Nothing
                else Just $ if s' > s then (s',i) else (s,fn)
    (separation, faceNormal) <- ifoldlM findEdgeWithMinPen (-1/0, 0) (b^.pVertices)
    -- Grab face's vertices
    let v1, v2 :: V2 Float
        v1 = (b^.pVertices) V.! faceNormal
        v2 = (b^.pVertices) V.! i2
        i2 = if faceNormal + 1 < V.length (b^.pVertices) then faceNormal + 1 else 0
    -- Check to see if center is within polygon
    let within :: Maybe Manifold
        within = let
            normal = negate $ b^.pU !* ((b^.pNormals) V.! faceNormal)
            contacts = (Just $ normal ^* penetration + a^.bPos, Nothing)
            penetration = a^.cRadius
            m = (mkManifold normal penetration) & mfContacts .~ contacts
            in if nearZero separation then Just m else Nothing
    -- Determine which vornoi regin of the edge center of circle lies within
    let dot1, dot2 :: Float
        dot1 = dot (center - v1) (v2 - v1)
        dot2 = dot (center - v2) (v1 - v2)
        penetration = a^.cRadius - separation
        rad2 = (a^.cRadius) ^ 2
    let closestToV1 = if qd center v1 > rad2 then Nothing else let
            normal = normalize $ (b^.pU) !* (v1 - center)
            contact = (b^.pU) !* v1 + b^.bPos
            in Just $ (mkManifold normal penetration) & mfContacts .~ (Just contact, Nothing)
    let closestToV2 = if qd center v2 > rad2 then Nothing else let
            normal = normalize $ (b^.pU) !* (v2 - center)
            contact = (b^.pU) !* v2 + b^.bPos
            in Just $ (mkManifold normal penetration) & mfContacts .~ (Just contact, Nothing)
    let n = (b^.pNormals) V.! faceNormal
    let closestToFace = if dot (center - v1) n > a^.cRadius then Nothing else let
            normal = negate $ (b^.pU) !* n
            contact = normal ^* (a^.cRadius) + a^.bPos
            in Just $ (mkManifold normal penetration) & mfContacts .~ (Just contact, Nothing)
    within <|> 
        (if | dot1 <= 0 -> closestToV1
            | dot2 <= 0 -> closestToV2
            | otherwise -> closestToFace)
{-# INLINE circleToPoly #-}

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

polyToPoly :: Body Poly -> Body Poly -> Maybe Manifold
polyToPoly a b = do
    -- Check for a separating axis with A's face planes
    let (faceA, penetrationA) = findAxisLeastPenetration a b
    guard $ penetrationA < 0 
    -- Check for a separating axis with B's face planes
    let (faceB, penetrationB) = findAxisLeastPenetration b a
    guard $ penetrationB < 0 
    -- Determine which shape contains reference face
    let (ref, inc, refIdx, flip) = if biasGreaterThan penetrationA penetrationB
            then (a, b, faceA, False)
            else (b, a, faceB, True)
    let refIdx' = if refIdx + 1 == V.length (ref^.pVertices) then 0 else refIdx + 1
    -- World space incident face
    let incFace = findIncidentFace ref inc refIdx
    let v1, v2 :: V2 Float
        v1 = (ref^.pVertices) V.! refIdx
        v2 = (ref^.pVertices) V.! refIdx'
    -- Transform vertices to world space
    let v1', v2' :: V2 Float
        v1' = (ref^.pU) !* v1 + ref^.bPos
        v2' = (ref^.pU) !* v2 + ref^.bPos
    -- Calculate reference face side normal in world space
    let sidePlaneNormal :: V2 Float
        sidePlaneNormal = v2' - v1'
    -- Orthogonalize
    let refFaceNormal = V2 (sidePlaneNormal^._y) (-sidePlaneNormal^._x)
    -- ax + by = c
    -- c is distance from origin
    let refC = dot refFaceNormal v1'
        negSide = negate $ dot sidePlaneNormal v1'
        posSide = dot sidePlaneNormal v2'
    -- Clip incident face to reference face side planes
    guard $ clip (-sidePlaneNormal) negSide incFace /= (Nothing,Nothing)
    guard $ clip sidePlaneNormal posSide incFace /= (Nothing,Nothing)
    -- Flip
    let normal = if flip then -refFaceNormal else refFaceNormal
    -- Keep points behinds reference face
    let separation = dot refFaceNormal (fst incFace) - refC
    let (penetration, mcontact) =
            if separation <= 0
            then (-separation, Just $ fst incFace)
            else (0, Nothing)
    let separation' = dot refFaceNormal (snd incFace) - refC
    let (penetration', mcontact') =
            if separation' <= 0
            then let
                p = penetration - separation'
                p' = p / if isJust mcontact then 2 else 1
                in (p', Just $ snd incFace)
            else (penetration, Nothing)
    if isNothing mcontact && isNothing mcontact'
    then Nothing
    else Just $ mkManifold normal penetration' & mfContacts .~ (mcontact, mcontact')
{-# INLINE polyToPoly #-}

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

bodyToBody :: (ToShape a, ToShape b) => Body a -> Body b -> Maybe Manifold
bodyToBody a b = b2b (fmap toShape a) (fmap toShape b)
 where
    b2b :: Body Shape -> Body Shape -> Maybe Manifold
    b2b a@Body{_bShape = ShapePoly ap} b@Body{_bShape = ShapePoly bp} = polyToPoly (ap <$ a) (bp <$ b)
    b2b a@Body{_bShape = ShapeCircle ac} b@Body{_bShape = ShapePoly bp} = circleToPoly (ac <$ a) (bp <$ b)
    b2b a@Body{_bShape = ShapeCircle ac} b@Body{_bShape = ShapeCircle bc} = circleToCircle (ac <$ a) (bc <$ b)

    b2b a b = b2b b a
--    b2b a@Body{_bShape = ShapeRect ar} b@Body{_bShape = ShapeRect br} = rectToRect (ar <$ a) (br <$ b)
--    b2b a@Body{_bShape = ShapeRect ar} b@Body{_bShape = ShapeCircle bc} = rectToCircle (ar <$ a) (bc <$ b)
