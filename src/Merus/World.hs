module Merus.World where

import Control.Lens
import Linear
import Merus.Types

-- Acceleration
--    F = mA
-- => A = F * 1/m
-- Explicit Euler
-- x += v * dt
-- v += (1/m * F) * dt
-- Semi-Implicit (Symplectic) Euler
-- v += (1/m * F) * dt
-- x += v * dt

integrateForces :: Float -> V2 Float -> Body a -> Body a
integrateForces dt gravity b@Body{..} = let
    vel :: V2 Float
    vel = (_bForce ^* _bInvMass + gravity) ^* (dt / 2)
    angVel = _bTorque * _bInvInertia * dt / 2
    in if _bInvMass == 0
        then b
        else b & (bVel +~ vel) -- . (bAngVel +~ angVel)

integrateVelocity :: Float -> V2 Float -> Body a -> Body a
integrateVelocity dt gravity b@Body{..} = let
    pos :: V2 Float
    pos = (_bVel ^* dt)
    orient = _bAngVel * dt
    b' = b & (bPos +~ pos) . (bOrient +~ orient)
    in if _bInvMass == 0 then b else integrateForces dt gravity b'
 where ppm = 50 -- pixels per meter

{-

Scene::Scene(f32 dt, uint32 iterations)
    : m_dt(dt)
    , m_iterations(iterations)
    {}

void Scene::Step()
{
  -- Generate new collision info
  contacts.clear();
  for(uint32 i = 0; i < bodies.size(); ++i)
  {
    Body *A = bodies[i];

    for(uint32 j = i + 1; j < bodies.size(); ++j)
    {
      Body *B = bodies[j];
      if(A->im == 0 && B->im == 0)
        continue;
      Manifold m;
      Manifold::Solve(A, i, B, j, m);
      if(m.contact_count)
        contacts.emplace_back(m);
    }
  }

  -- Integrate forces
  for(uint32 i = 0; i < bodies.size(); ++i)
    IntegrateForces(bodies[i], m_dt);

  -- Initialize collision
  for(uint32 i = 0; i < contacts.size(); ++i) {
    Manifold &contact = contacts[i];
    contact.Initialize(bodies[contact.a], bodies[contact.b]);
  }

  -- Solve collisions
  for(uint32 j = 0; j < m_iterations; ++j)
    for(uint32 i = 0; i < contacts.size(); ++i) {
    Manifold &contact = contacts[i];
    contact.ApplyImpulse(bodies[contact.a], bodies[contact.b]);
  }

  -- Integrate velocities
  for(uint32 i = 0; i < bodies.size(); ++i)
    IntegrateVelocity(bodies[i], m_dt);

  -- Correct positions
  for(uint32 i = 0; i < contacts.size(); ++i) {
    Manifold &contact = contacts[i];
    contact.PositionalCorrection(bodies[contact.a], bodies[contact.b]);
  }

  -- Clear all forces
  for(uint32 i = 0; i < bodies.size(); ++i)
  {
    Body *b = bodies[i];
    b->force.SetXY(0, 0);
    b->torque = 0;
  }
}

void Scene::Render()
{
  for(uint32 i = 0; i < bodies.size(); ++i)
  {
    Body *b = bodies[i];
    b->shape.Draw(b);
  }

  glPointSize(4.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.0f, 0.0f);
  for(uint32 i = 0; i < contacts.size(); ++i)
  {
    Manifold& m = contacts[i];
    for(uint32 j = 0; j < m.contact_count; ++j)
    {
      Vec2 c = m.contacts[j];
      glVertex2f(c.x, c.y);
    }
  }
  glEnd();
  glPointSize(1.0f);

  glBegin(GL_LINES);
  glColor3f(0.0f, 1.0f, 0.0f);
  for(uint32 i = 0; i < contacts.size(); ++i)
  {
    Manifold& m = contacts[i];
    Vec2 n = m.normal;
    for(uint32 j = 0; j < m.contact_count; ++j)
    {
      Vec2 c = m.contacts[j];
      glVertex2f(c.x, c.y);
      n.withScalarMul(0.75f);
      c += n;
      glVertex2f(c.x, c.y);
    }
  }
  glEnd();
}

Body *Scene::Add(Shape shape, uint32 x, uint32 y)
{
  Body *b = new Body(shape, x, y);
  bodies.push_back(b);
  return b;
}
-}
