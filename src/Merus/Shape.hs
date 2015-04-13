module Merus.Shape where

import qualified Data.Vector.Storable as V
import qualified Data.Foldable as F
import Control.Lens
import Linear
import Merus.Types
import Merus.Math

computeCircleMass :: Float -> Body Circle -> Body Circle
computeCircleMass density a = let
    r2 = (a^.cRadius) ^ 2
    m = pi * r2 * density
    i = m * r2 
    in a & (mass .~ m) . (inertia .~ i)

computePolygonMass :: Float -> Body Poly -> Body Poly
computePolygonMass density a = let
    kInv3 = recip 3
    iter (area, i, c) i1 = let
        p1 = (a^.pVertices) V.! i1
        i2 = if i1 + 1 < V.length (a^.pVertices) then i1 + 1 else 0
        p2 = (a^.pVertices) V.! i2
        d = xrossv p1 p2
        triArea = 0.5 * d
        area' = area + triArea
        c' = (p1 + p2) ^* (triArea * kInv3)
        intx2 = (p1^._x)^2 + p1^._x * p2^._x + (p2^._x)^2
        inty2 = (p1^._y)^2 + p1^._y * p2^._y + (p2^._y)^2
        i' = 0.25 * kInv3 * d * (intx2 + inty2)
        in (area', i', c')
    (area, i, c) = F.foldl' iter (0, 0, zero) [0..V.length (a^.pVertices) - 1]
    c' = c ^/ area
    verts = V.map (subtract c') (a^.pVertices)
    --
    m = density * area
    i' = density * i 
    in a & (mass .~ m) . (inertia .~ i') . (pVertices .~ verts)

polySetBox :: Float -> Float -> Poly -> Poly
polySetBox hw hh p = let
    verts = V.fromListN 4 [
            V2 (-hw) hh,
            V2 hw (-hh),
            V2 hw hh,
            V2 (-hw) hh
        ]
    norms = V.fromListN 4 [
            V2 0 (-1),
            V2 1 0,
            V2 0 1,
            V2 (-1) 0
        ]
    in p & (pVertices .~ verts) . (pNormals .~ norms)

polyEmpty :: Poly
polyEmpty = Poly identity V.empty V.empty

mkBody :: a -> Body a
mkBody = Body Dynamic nil nil nil nil nil nil nil nil nil nil nil nil nil
 where
    nil :: Num a => a
    nil = fromInteger 0

mkSquare' :: Float -> Body Poly
mkSquare' s = mkBody $ polySetBox s s polyEmpty

mkRect' :: V2 Float -> Body Poly
mkRect' (V2 x y) = mkBody $ polySetBox x y polyEmpty

mkPoly' :: V.Vector (V2 Float) -> V.Vector (V2 Float) -> Body Poly
mkPoly' verts norms = mkBody $ polyEmpty & (pVertices .~ verts) . (pNormals .~ norms)

mkCircle' :: Float -> Body Circle
mkCircle' = mkBody . Circle

mkRect :: V2 Float -> Body Shape
mkRect = fmap toShape . mkRect'

mkSquare :: Float -> Body Shape
mkSquare = fmap toShape . mkSquare'

mkPoly :: V.Vector (V2 Float) -> V.Vector (V2 Float) -> Body Shape
mkPoly verts norms = fmap toShape $ mkPoly' verts norms

mkCircle :: Float -> Body Shape
mkCircle = fmap toShape . mkCircle'
