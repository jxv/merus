module Merus.Math where

import Control.Lens
import Linear

recipNoInf :: (Fractional a, Epsilon a) => a -> a 
recipNoInf x = if nearZero x then 0 else (1 / x)

xrossv :: V2 Float -> V2 Float -> Float
xrossv a b = a^._x * b^._y - a^._y * b^._x
{-# INLINE xrossv #-}

xrossf :: V2 Float -> Float -> V2 Float
xrossf v a = V2 ((v^._y) * a) ((v^._x) * (-a))
{-# INLINE xrossf #-}

xrossf' :: Float -> V2 Float -> V2 Float
xrossf' a v = V2 ((v^._y) * (-a)) ((v^._x) * a)
{-# INLINE xrossf' #-}
