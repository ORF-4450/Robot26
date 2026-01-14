package Team4450.Robot26.utility;

public class VisionBuffer {
    public VisionPose lastNode;

    public VisionBuffer() {
    }

    public VisionPose getLatest() {
        return lastNode;
    }

    public VisionPose get(int targetIndex) {
        int i = 1;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            if (i == targetIndex) {
                return node;
            }
            node = node.nextNode;
            i++;
        }
        return null;
    }

    public int length() {
        int i = 1;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            node = node.nextNode;
            i++;
        }
        return i;
    }

    public void append(VisionPose poseToAdd) {
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            node = node.nextNode;
        }
        node.nextNode = poseToAdd;
    }

    public void truncate(int len) {
        int i = 1;
        VisionPose node = lastNode;
        while (node.nextNode != null) {
            if (i > len) {
                node.nextNode = null;
                return;
            }
            node = node.nextNode;
            i++;
        }
    }
}
