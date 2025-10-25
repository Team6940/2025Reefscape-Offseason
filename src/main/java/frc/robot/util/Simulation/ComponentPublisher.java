package frc.robot.util.Simulation;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class ComponentPublisher {
    private final StructArrayPublisher<Pose3d> publisher;

    public ComponentPublisher(String topicName) {
        publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("AdvantageKit/Components/"+topicName, Pose3d.struct)
            .publish();
        // Logger.recordOutput(topicName, new Pose3d[0]);
    }

    /**Piblish single component pose.
     * 
     * @param pose
     */
    public void setPose(Pose3d pose) {
        publisher.set(new Pose3d[]{pose});
    }


    /** Publish multiple component poses.
     * 
     * @param poses
     */
    public void setPoses(Pose3d[] poses) {
        publisher.set(poses);       //Note: need to preset pose3d 
    }
}
