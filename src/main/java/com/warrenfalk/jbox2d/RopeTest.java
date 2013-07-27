package com.warrenfalk.jbox2d;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.RopeJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class RopeTest extends TestbedTest {
	
	class Rope {
		public Body head;
		public Body tail;
	}
	
	final static int CHAIN = 0x2; // chains collide with the world, but chains don't collide with chains

	@Override
	public void initTest(boolean deserialized) {
		setTitle("Chain test");
		
		getWorld().setGravity(new Vec2(0, -10f));
		
		PolygonShape baseShape = new PolygonShape();
		baseShape.setAsBox(4f, 2f);
		PolygonShape weightShape = new PolygonShape();
		weightShape.setAsBox(3f, 1f);
		
		BodyDef fixed = new BodyDef();
		fixed.type = BodyType.STATIC;
		
		BodyDef dynamic = new BodyDef();
		dynamic.type = BodyType.DYNAMIC;
		
		FixtureDef fd = new FixtureDef();
		fd.shape = baseShape;
		fd.density = 1.0f;
		fd.filter.maskBits = 0; // doesn't collide with anything

		fixed.position.set(0, 0);
		Body base = m_world.createBody(fixed);
		base.createFixture(fd);
		
		fixed.position.set(0, -5);
		Body obstacle = getWorld().createBody(fixed);
		fd.filter.maskBits = 0xFFFF; // collide with everything
		obstacle.createFixture(fd);
		
		float thickness = 0.2f;
		float resolution = 0.6f;
		Rope rope = CreateRope(thickness, resolution, new Vec2(0, 0), new Vec2(resolution * 30f, 0));
		
		RevoluteJointDef jd = new RevoluteJointDef();
		jd.initialize(base, rope.head, new Vec2());
		jd.collideConnected = false;
		getWorld().createJoint(jd);
		
		dynamic.position = new Vec2(rope.tail.getPosition());
		dynamic.position.addLocal(new Vec2(3f, 0));
		Body weight = getWorld().createBody(dynamic);
		fd.shape = weightShape;
		fd.density = 20f;
		fd.filter.maskBits = 0xFFFF & ~CHAIN;
		fd.filter.categoryBits = CHAIN; // the thing on the end of the chain collides like the chain
		weight.createFixture(fd);
		
		jd.initialize(weight, rope.tail, rope.tail.getPosition().add(new Vec2(0f, 0)));
		jd.collideConnected = false;
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
		
		Body[] links = new Body[count];
		
		int i;
		for (i = 0; i < count; i++) {
			BodyDef bd = new BodyDef();
			bd.linearDamping = 0.999f;
			bd.type = BodyType.DYNAMIC;
			bd.angularDamping = 0.999f;
			float x = from.x + segVector.x * resolution * i;
			float y = from.y + segVector.y * resolution * i;
			bd.position.set(x + (resolution * 0.5f), y);
			
			FixtureDef fd = new FixtureDef();
			fd.shape = linkShape;
			fd.restitution = 0f;
			fd.density = 9f;
			fd.filter.categoryBits = CHAIN;
			fd.filter.maskBits = 0xFFFF & ~CHAIN; // collide with anything, except other chains
			
			Body link = getWorld().createBody(bd);
			links[i] = link;
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
		
		for (i = 3; i < (count - 3); i += 2) {
			RopeJointDef rj = new RopeJointDef();
			rj.bodyA = rope.tail;
			rj.bodyB = links[i];
			rj.localAnchorA.set(0, 0);
			rj.localAnchorB.set(0, 0);
			rj.maxLength = rj.bodyA.getPosition().sub(rj.bodyB.getPosition()).length();
			rj.collideConnected = false;
			getWorld().createJoint(rj);
			
			rj = new RopeJointDef();
			rj.bodyA = rope.head;
			rj.bodyB = links[i];
			rj.localAnchorA.set(0, 0);
			rj.localAnchorB.set(0, 0);
			rj.maxLength = rj.bodyA.getPosition().sub(rj.bodyB.getPosition()).length();
			rj.collideConnected = false;
			getWorld().createJoint(rj);
		}
		
		RopeJointDef rj = new RopeJointDef();
		rj.bodyA = rope.head;
		rj.bodyB = rope.tail;
		rj.localAnchorA.set(0, 0);
		rj.localAnchorB.set(0, 0);
		rj.maxLength = rope.head.getPosition().sub(rope.tail.getPosition()).length();
		rj.collideConnected = false;
		getWorld().createJoint(rj);
		
		return rope;
		
	}

	@Override
	public String getTestName() {
		return "Chain";
	}

}
