b2Vec2 g(0, -10);
m_world->SetGravity(g);
b2Body** bodies = (b2Body**)b2Alloc(4 * sizeof(b2Body*));
b2Joint** joints = (b2Joint**)b2Alloc(1 * sizeof(b2Joint*));
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(-2.09463739, -1.78226948);
  bd.angle = 0.49747932;
  bd.linearVelocity.Set(-2.65783191, -2.75087976);
  bd.angularVelocity = 0.157078803;
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.allowSleep = bool(4);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.enabled = bool(32);
  bd.gravityScale = 1;
  bodies[0] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 0.200000003;
    fd.restitution = 0;
    fd.restitutionThreshold = 1;
    fd.density = 1;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-0.125, -0.125);
    vs[1].Set(0.125, -0.125);
    vs[2].Set(0.125, 0.125);
    vs[3].Set(-0.125, 0.125);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[0]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(2);
  bd.position.Set(0, 10);
  bd.angle = 0.497418433;
  bd.linearVelocity.Set(-1.00045121e-07, 2.76810308e-08);
  bd.angularVelocity = 0.157079637;
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.allowSleep = bool(0);
  bd.awake = bool(2);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.enabled = bool(32);
  bd.gravityScale = 1;
  bodies[1] = m_world->CreateBody(&bd);

  {
    b2FixtureDef fd;
    fd.friction = 0.200000003;
    fd.restitution = 0;
    fd.restitutionThreshold = 1;
    fd.density = 5;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-10, -10.5);
    vs[1].Set(10, -10.5);
    vs[2].Set(10, -9.5);
    vs[3].Set(-10, -9.5);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 0.200000003;
    fd.restitution = 0;
    fd.restitutionThreshold = 1;
    fd.density = 5;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-10, 9.5);
    vs[1].Set(10, 9.5);
    vs[2].Set(10, 10.5);
    vs[3].Set(-10, 10.5);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 0.200000003;
    fd.restitution = 0;
    fd.restitutionThreshold = 1;
    fd.density = 5;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(-10.5, -10);
    vs[1].Set(-9.5, -10);
    vs[2].Set(-9.5, 10);
    vs[3].Set(-10.5, 10);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
  {
    b2FixtureDef fd;
    fd.friction = 0.200000003;
    fd.restitution = 0;
    fd.restitutionThreshold = 1;
    fd.density = 5;
    fd.isSensor = bool(0);
    fd.filter.categoryBits = uint16(1);
    fd.filter.maskBits = uint16(65535);
    fd.filter.groupIndex = int16(0);
    b2PolygonShape shape;
    b2Vec2 vs[8];
    vs[0].Set(9.5, -10);
    vs[1].Set(10.5, -10);
    vs[2].Set(10.5, 10);
    vs[3].Set(9.5, 10);
    shape.Set(vs, 4);

    fd.shape = &shape;

    bodies[1]->CreateFixture(&fd);
  }
}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0, 0);
  bd.angle = 0;
  bd.linearVelocity.Set(0, 0);
  bd.angularVelocity = 0;
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.enabled = bool(32);
  bd.gravityScale = 1;
  bodies[2] = m_world->CreateBody(&bd);

}
{
  b2BodyDef bd;
  bd.type = b2BodyType(0);
  bd.position.Set(0, 0);
  bd.angle = 0;
  bd.linearVelocity.Set(0, 0);
  bd.angularVelocity = 0;
  bd.linearDamping = 0;
  bd.angularDamping = 0;
  bd.allowSleep = bool(4);
  bd.awake = bool(0);
  bd.fixedRotation = bool(0);
  bd.bullet = bool(0);
  bd.enabled = bool(32);
  bd.gravityScale = 1;
  bodies[3] = m_world->CreateBody(&bd);

}
{
  b2RevoluteJointDef jd;
  jd.bodyA = bodies[2];
  jd.bodyB = bodies[1];
  jd.collideConnected = bool(0);
  jd.localAnchorA.Set(0, 10);
  jd.localAnchorB.Set(0, 0);
  jd.referenceAngle = 0;
  jd.enableLimit = bool(0);
  jd.lowerAngle = 0;
  jd.upperAngle = 0;
  jd.enableMotor = bool(1);
  jd.motorSpeed = 0.157079637;
  jd.maxMotorTorque = 100000000;
  joints[0] = m_world->CreateJoint(&jd);
}
b2Free(joints);
b2Free(bodies);
joints = nullptr;
bodies = nullptr;
