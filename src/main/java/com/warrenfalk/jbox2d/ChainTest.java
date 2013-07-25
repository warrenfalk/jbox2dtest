package com.warrenfalk.jbox2d;

import org.jbox2d.collision.shapes.CircleShape;
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
		circle.m_radius = 0.4f;
		
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		
		
		Body prev = null;
		for (int i = 0; i < 15; i++) {
			FixtureDef fd = new FixtureDef();
			fd.shape = circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;
			
			bd.position.set(-9f + 1.2f * i, 0.35f);
			Body link = m_world.createBody(bd);
			link.createFixture(fd);
			
			if (prev != null) {
				DistanceJointDef djd = new DistanceJointDef();
				djd.initialize(prev, link, prev.getPosition(), link.getPosition());
				getWorld().createJoint(djd);
			}
			
			prev = link;
		}
	}

	@Override
	public String getTestName() {
		return "Chain";
	}

}
