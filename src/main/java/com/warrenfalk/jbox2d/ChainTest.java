package com.warrenfalk.jbox2d;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.testbed.framework.TestbedTest;

public class ChainTest extends TestbedTest {

	@Override
	public void initTest(boolean deserialized) {
		setTitle("Chain test");
		
		//getWorld().setGravity(new Vec2());

		CircleShape circle = new CircleShape();
		circle.m_radius = 0.2f;
		
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		
		BodyDef fixed = new BodyDef();
		fixed.type = BodyType.STATIC;
		
		FixtureDef fd = new FixtureDef();
		fd.shape = circle;
		fd.density = 2.0f;
		fd.friction = 0.9f;
		
		fixed.position.set(-10.2f, 0.35f);
		Body base = m_world.createBody(fixed);
		base.createFixture(fd);
		
		Body prev = base;
		for (int i = 0; i < 30; i++) {
			
			bd.position.set(-9f + 0.6f * i, 0.35f);
			Body link = m_world.createBody(bd);
			link.createFixture(fd);
			
			if (prev != null) {
				DistanceJointDef djd = new DistanceJointDef();
				djd.initialize(prev, link, prev.getPosition(), link.getPosition());
				getWorld().createJoint(djd);
			}
			
			prev = link;
		}
		
		bd.position.set(10.2f, 0.35f);
		fd.density = 100f;
		Body weight = m_world.createBody(bd);
		weight.createFixture(fd);
		
		DistanceJointDef djd = new DistanceJointDef();
		djd.initialize(prev, weight, prev.getPosition(), weight.getPosition());
		getWorld().createJoint(djd);
		
		PolygonShape ps = new PolygonShape();
		ps.setAsBox(2.5f, 0.8f);

		fixed.position.set(3f, -4f);
		Body block = getWorld().createBody(fixed);
		
		fd.shape = ps;
		block.createFixture(fd);
		
		
		

	}

	@Override
	public String getTestName() {
		return "Chain";
	}

}
