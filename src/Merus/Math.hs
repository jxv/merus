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

clamp :: Ord a => a -> a -> a -> a
clamp low high = max low . min high
{-# INLINE clamp #-}

clampF :: (Applicative f, Ord a) => f a -> f a -> f a -> f a
clampF low high n = max <$> low <*> (min <$> high <*> n)
{-# INLINE clampF #-}

biasGreaterThan :: Float -> Float -> Bool
biasGreaterThan a b = let
    biasRelative = 0.95
    biasAbsolute = 0.01
    in a >= b * biasRelative + a * biasAbsolute
