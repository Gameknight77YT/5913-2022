package frc.robot.pathPlanner.org.json.simple.parser;

import java.util.List;
import java.util.Map;

public interface ContainerFactory {
	Map createObjectContainer();

	List creatArrayContainer();
}
