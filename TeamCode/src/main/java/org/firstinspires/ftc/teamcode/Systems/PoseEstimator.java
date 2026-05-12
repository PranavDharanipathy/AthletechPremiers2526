package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PedroPathing.PoseVelocity;

/// Kalman Filter Pose estimator for fusing odometry and camera pose data
public class PoseEstimator {

    // State vector: [x, y, theta, vx, vy, vtheta] (pose, velocities)
    private RealVector state;
    private RealMatrix covariance;

    // Process noise covariance (uncertainty in motion model)
    private RealMatrix Q;
    private double[] processNoiseStdDev;

    // Measurement noise covariances
    private RealMatrix R_odometry;
    private RealMatrix R_camera;

    // State transition matrix - assumes constant velocity model
    private RealMatrix F;

    // Measurement matrix (x, y, theta)
    private RealMatrix H;

    private long lastUpdateTime;

    /// @param processNoiseStdDev Standard deviation of process noise (x, y, theta, vx, vy, vtheta)
    public void setProcessNoiseStdDev(double[] processNoiseStdDev) {
        this.processNoiseStdDev = processNoiseStdDev;
    }

    /**
     * @param odometryStdDev Standard deviation of odometry measurements [x_std, y_std, theta_std]
     * @param cameraStdDev Standard deviation of camera measurements [x_std, y_std, theta_std]
     * @param processNoiseStdDev Standard deviation of process noise [x_std, y_std, theta_std, vx_std, vy_std, vtheta_std]
     */
    public PoseEstimator(
            Pose pose,
            double[] odometryStdDev,
            double[] cameraStdDev,
            double[] processNoiseStdDev) {

        final double initialX = pose.getX(); //x state (initial)
        final double initialY = pose.getY(); //y state (initial)
        final double initialTheta = pose.getHeading(); //heading state (initial)

        this.processNoiseStdDev = processNoiseStdDev;
        this.lastUpdateTime = System.currentTimeMillis();

        // Initialize state: [x, y, theta, vx, vy, vtheta]
        this.state = new ArrayRealVector(new double[]{
                initialX, initialY, initialTheta, 0, 0, 0
        });

        // Initialize covariance - start with uncertainty in position/orientation
        this.covariance = new Array2DRowRealMatrix(6, 6);
        for (int i = 0; i < 6; i++) {
            covariance.setEntry(i, i, processNoiseStdDev[i] * processNoiseStdDev[i]);
        }

        // Measurement matrix H (we measure x, y, theta only)
        this.H = new Array2DRowRealMatrix(new double[][]{
                {1, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0},
                {0, 0, 1, 0, 0, 0}
        });

        // Process noise covariance Q
        this.Q = new Array2DRowRealMatrix(6, 6);
        for (int i = 0; i < 6; i++) {
            Q.setEntry(i, i, processNoiseStdDev[i] * processNoiseStdDev[i]);
        }

        // Measurement noise covariance for odometry (3x3: x, y, theta)
        this.R_odometry = new Array2DRowRealMatrix(new double[][]{
                {odometryStdDev[0] * odometryStdDev[0], 0, 0},
                {0, odometryStdDev[1] * odometryStdDev[1], 0},
                {0, 0, odometryStdDev[2] * odometryStdDev[2]}
        });

        // Measurement noise covariance for camera (3x3: x, y, theta)
        this.R_camera = new Array2DRowRealMatrix(new double[][]{
                {cameraStdDev[0] * cameraStdDev[0], 0, 0},
                {0, cameraStdDev[1] * cameraStdDev[1], 0},
                {0, 0, cameraStdDev[2] * cameraStdDev[2]}
        });
    }

    /**
     * Predict step - propagate state forward in time
     */
    private void predict(double dt) {

        RealMatrix F = new Array2DRowRealMatrix(new double[][]{
                {1, 0, 0, dt, 0, 0},
                {0, 1, 0, 0, dt, 0},
                {0, 0, 1, 0, 0, dt},
                {0, 0, 0, 1, 0, 0},
                {0, 0, 0, 0, 1, 0},
                {0, 0, 0, 0, 0, 1}
        });

        // x = F * x
        state = F.operate(state);

        // P = F * P * F^T + Q
        RealMatrix FT = F.transpose();
        covariance = F.multiply(covariance).multiply(FT).add(Q);

        normalizeAngleRad();
    }

    /// The two inputs of {@link Follower} and {@link Camera} must have been updated
    /// before this method is called.
    public void update(Follower odometry, Camera camera) {

        //calculate dt in seconds (clamped to limit prediction range)
        long currentTime = System.currentTimeMillis();
        double dt = MathUtil.clamp((currentTime - lastUpdateTime) / 1000d, 0.001, 0.3);
        lastUpdateTime = currentTime;

        predict(dt);

        updateWithOdometry(odometry.getPose());

        if (camera.canUseMT2Pose()) updateWithCamera(camera.getBotPoseMT2());
    }

    private void updateWithOdometry(Pose odometryPose) {
        updateWithMeasurement(odometryPose.getX(), odometryPose.getY(), odometryPose.getHeading(), R_odometry);
    }

    private void updateWithCamera(Pose cameraPose) {
        updateWithMeasurement(cameraPose.getX(), cameraPose.getY(), cameraPose.getHeading(), R_camera);
    }

    // General update step for any measurement
    private void updateWithMeasurement(double measuredX, double measuredY, double measuredTheta, RealMatrix R) {
        // Measurement vector
        RealVector z = new ArrayRealVector(new double[]{measuredX, measuredY, measuredTheta});

        // Normalize angle difference
        double angleDiff = measuredTheta - state.getEntry(2);
        angleDiff = MathUtil.normalizeAngleRad(angleDiff);
        z.setEntry(2, state.getEntry(2) + angleDiff);

        // Innovation (measurement residual)
        // y = z - H * x
        RealVector y = z.subtract(H.operate(state));

        // Normalize angle component of innovation
        y.setEntry(2, MathUtil.normalizeAngleRad(y.getEntry(2)));

        // Innovation covariance
        // S = H * P * H^T + R
        RealMatrix HT = H.transpose();
        RealMatrix S = H.multiply(covariance).multiply(HT).add(R);

        // Kalman gain
        // K = P * H^T * S^-1
        RealMatrix inverseS = invertMatrix(S);
        RealMatrix K = covariance.multiply(HT).multiply(inverseS);

        // Update state
        // x = x + K * y
        state = state.add(K.operate(y));

        // Update covariance
        // P = (I - K * H) * P
        RealMatrix I = createIdentityMatrix(6);
        RealMatrix IMinusKH = I.subtract(K.multiply(H));
        covariance = IMinusKH.multiply(covariance);

        // Normalize angle to [-pi, pi]
        normalizeAngleRad();
    }

    public Pose getPose() {

        return new Pose(
                state.getEntry(0),
                state.getEntry(1),
                state.getEntry(2)
        );
    }
    public PoseVelocity getPoseVelocity() {

        return new PoseVelocity(
                state.getEntry(3),
                state.getEntry(4),
                state.getEntry(5)
        );
    }


    /// Get the full state vector including velocities
    /// @return (x, y, theta, vx, vy, vtheta)
    public double[] getFullState() {
        return state.toArray();
    }

    /// Get the position uncertainty (standard deviation)
    /// @return x, y, theta as {@link Pose}
    public Pose getUncertainty() {
        return new Pose(
                Math.sqrt(covariance.getEntry(0, 0)),
                Math.sqrt(covariance.getEntry(1, 1)),
                Math.sqrt(covariance.getEntry(2, 2))
        );
    }

    /// Reset the filter to a known state
    public void reset(Pose pose) {

        state = new ArrayRealVector(new double[]{pose.getX(), pose.getY(), pose.getHeading(), 0, 0, 0});

        covariance = new Array2DRowRealMatrix(6, 6);
        for (int i = 0; i < 6; i++) {
            covariance.setEntry(i, i, processNoiseStdDev[i] * processNoiseStdDev[i]);
        }

        lastUpdateTime = System.currentTimeMillis();
    }

    /// Normalize angle to [-pi, pi] range
    private void normalizeAngleRad() {
        double theta = state.getEntry(2);
        state.setEntry(2, MathUtil.normalizeAngleRad(theta));
    }

    private RealMatrix createIdentityMatrix(int size) {

        RealMatrix identityMatrix = new Array2DRowRealMatrix(size, size);
        for (int i = 0; i < size; i++) {
            identityMatrix.setEntry(i, i, 1d);
        }
        return identityMatrix;
    }

    /// Uses Gaussian elimination
    private RealMatrix invertMatrix(RealMatrix matrix) {
        int n = matrix.getRowDimension();
        double[][] data = matrix.getData();
        double[][] inverse = new double[n][n];

        //create augmented matrix
        double[][] augmented = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = data[i][j];
            }
            augmented[i][n + i] = 1d;
        }

        //Gaussian elimination
        for (int i = 0; i < n; i++) {

            //finds pivot
            double maxVal = Math.abs(augmented[i][i]);
            int maxRow = i;
            for (int k = i + 1; k < n; k++) {
                if (Math.abs(augmented[k][i]) > maxVal) {
                    maxVal = Math.abs(augmented[k][i]);
                    maxRow = k;
                }
            }

            double[] temp = augmented[i];
            augmented[i] = augmented[maxRow];
            augmented[maxRow] = temp;

            double divisor = augmented[i][i];
            for (int j = 0; j < 2 * n; j++) {
                augmented[i][j] /= divisor;
            }

            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = augmented[k][i];
                    for (int j = 0; j < 2 * n; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }

        //getting inverse
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                inverse[i][j] = augmented[i][n + j];
            }
        }

        return new Array2DRowRealMatrix(inverse);
    }
}