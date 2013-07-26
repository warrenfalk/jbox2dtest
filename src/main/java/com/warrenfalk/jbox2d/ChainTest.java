package com.warrenfalk.jbox2d;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class ChainTest extends TestbedTest {
	
	class Rope {
		public Body head;
		public Body tail;
	}

	@Override
	public void initTest(boolean deserialized) {
		setTitle("Chain test");
		
		PolygonShape baseShape = new PolygonShape();
		baseShape.setAsBox(4f, 2f);
		PolygonShape weightShape = new PolygonShape();
		weightShape.setAsBox(1f, 1f);
		
		BodyDef fixed = new BodyDef();
		fixed.type = BodyType.STATIC;
		
		BodyDef dynamic = new BodyDef();
		dynamic.type = BodyType.DYNAMIC;
		
		FixtureDef fd = new FixtureDef();
		fd.shape = baseShape;
		fd.density = 1.0f;
		fd.filter.maskBits = 0;

		fixed.position.set(0, 0);
		Body base = m_world.createBody(fixed);
		base.createFixture(fd);
		
		float thickness = 0.2f;
		float resolution = 0.6f;
		Rope rope = CreateRope(thickness, resolution, new Vec2(0, 0), new Vec2(resolution * 30f, 0));
		
		RevoluteJointDef jd = new RevoluteJointDef();
		jd.initialize(base, rope.head, fixed.position);
		jd.collideConnected = false;
		getWorld().createJoint(jd);
		
		dynamic.position = rope.tail.getPosition(); 
		Body weight = getWorld().createBody(dynamic);
		fd.shape = weightShape;
		fd.density = 10f;
		fd.filter.maskBits = 0;
		weight.createFixture(fd);
		
		jd.initialize(rope.tail, weight, weight.getPosition());
		getWorld().createJoint(jd);

	}

	private Rope CreateRope(float thickness, float resolution, Vec2 from, Vec2 to) {
		Vec2 vector = to.sub(from);
		float length = vector.length();
		int count = (int)Math.ceil(length / resolution);
		
		Vec2 segVector = new Vec2(vector);
		segVector.normalize();
		
		PolygonShape linkShape = new PolygonShape();
		linkShape.setAsBox(resolution / 2f, thickness / 2f);
		
		Rope rope = new Rope();
		
		int i;
		for (i = 0; i < count; i++) {
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			float x = from.x + segVector.x * resolution * i;
			float y = from.y + segVector.y * resolution * i;
			bd.position.set(x + (resolution * 0.5f), y); 
			
			FixtureDef fd = new FixtureDef();
			fd.shape = linkShape;
			fd.density = 1.0f;
			
			Body link = getWorld().createBody(bd);
			link.createFixture(fd);
			
			if (i == 0) {
				rope.head = link;
			}
			else {
				RevoluteJointDef jd = new RevoluteJointDef();
				jd.initialize(rope.tail, link, new Vec2(x, y));
				getWorld().createJoint(jd);
			}
			rope.tail = link;
		}
		
		RopeJointDef rj = new RopeJointDef();
		rj.bodyA = rope.head;
		rj.bodyB = rope.tail;
		rj.localAnchorA.set(0, 0);
		rj.localAnchorB.set(0, 0);
		rj.maxLength = length;
		rj.collideConnected = false;
		getWorld().createJoint(rj);
		
		return rope;
		
		/*
		
		PolygonShape linkShape = new PolygonShape();
		linkShape.setAsBox(0.6f, 0.1f);
		
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		
		FixtureDef fd = new FixtureDef();
		fd.shape = linkShape;
		fd.density = 2.0f;
		fd.friction = 0.9f;
		
		
		RopeJointDef rjd = new RopeJointDef();
		rjd.bodyA = base;
		rjd.localAnchorA.setZero();
		
		Body prev = base;
		int i;
		for (i = 0; i < 30; i++) {
			
			bd.position.set(-9f + 0.3f * i, 0.35f);
			Body link = m_world.createBody(bd);
			link.createFixture(fd);
			
			if (prev != null) {
				RevoluteJointDef jd = new RevoluteJointDef();
				Vec2 anchor = new Vec2(prev.getPosition());
				anchor.x += 0.1f;
				jd.initialize(prev, link, anchor);
				getWorld().createJoint(jd);
				
				rjd.bodyB = link;
				rjd.localAnchorB.setZero();
				rjd.maxLength = 0.3f * i;
				//getWorld().createJoint(rjd);
			}
			
			prev = link;
		}
		
		PolygonShape ps = new PolygonShape();
		ps.setAsBox(2.5f, 0.8f);
		
		bd.position.set(-9f + 0.3f * i, 0.35f);
		fd.shape = ps;
		fd.density = 100f;
		Body weight = m_world.createBody(bd);
		weight.createFixture(fd);
		
		DistanceJointDef djd = new DistanceJointDef();
		djd.initialize(prev, weight, prev.getPosition(), weight.getPosition());
		getWorld().createJoint(djd);
		
		rjd.bodyB = weight;
		rjd.localAnchorB.setZero();
		rjd.maxLength = bd.position.x - fixed.position.x;
		getWorld().createJoint(rjd);

		

		fixed.position.set(-3f, -4f);
		Body block = getWorld().createBody(fixed);
		
		fd.shape = ps;
		block.createFixture(fd);
		*/
		
	}

	@Override
	public String getTestName() {
		return "Chain";
	}

}
