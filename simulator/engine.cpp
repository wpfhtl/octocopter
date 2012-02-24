#include "engine.h"

Engine::Engine(void) : QObject()
{
//    mPose = pose;
//    mRotationDirection = rotationDirection;

    initializePropellers();

    // According to http://www.mikrokopter.de/ucwiki/ROXXY2827-35, with
    // a 10*EPP our motor does 820g of thrust = 8 Newton at 10A.

    Q_ASSERT(setPropeller("APC_SF_1047"));

    qDebug() << "Engine::Engine(): thrust at 10 Amps:" << calculateThrust(10.0);

    // Try to match http://gallery.mikrokopter.de/main.php?g2_view=core.DownloadItem&g2_itemId=49297
    Q_ASSERT(calculateThrust(10.0) > 7.7 && calculateThrust(10.0) < 10);
}

Engine::Engine(const Engine &other)
{
    mPose = other.mPose;
    mPropellers = other.mPropellers;
    mCurrentPropeller = other.mCurrentPropeller;
}

Engine& Engine::operator=(const Engine &other)
{
    mPose = other.mPose;
    mPropellers = other.mPropellers;
    mCurrentPropeller = other.mCurrentPropeller;

    return *this;
}

bool Engine::setPropeller(const QString &propeller)
{
    if(mPropellers.contains(propeller))
    {
        mCurrentPropeller = mPropellers.value(propeller);
        return true;
    }

    return false;
}

/*
btVector3 Engine::calculateThrust(const int rpm) const
{
    // This method calculates the thrust caused by rotating the propeller at @rpm in newton.
    // As the engine/rotor-combination doesn't need to be mounted pointing straight up,
    // we return a vector of thrust, depending on @rpm and @mPose.

    Q_ASSERT(mCurrentPropeller.c1 != 0 && mCurrentPropeller.c2 != 0 && mCurrentPropeller.c3 != 0);
//    Q_ASSERT(rpm > mCurrentPropeller.rpmMin && rpm < mCurrentPropeller.rpmMax);

    float thrust = (mCurrentPropeller.c3 * pow(rpm, 2) + mCurrentPropeller.c2 * abs(rpm) + mCurrentPropeller.c1) / (1000.0 / 9.81);
    if(rpm < 0) thrust *= -1.0; // invert the thrust if we rotate backwards

    const btQuaternion rotation = mPose.getRotation();

    const btVector3 thrustVector = btVector3(0, thrust, 0).rotate(rotation.getAxis(), rotation.getAngle());
//    qDebug() << "Engine::calculateThrust(): rpm" << rpm << "thrust" << thrust << "vectorized for angle " << rad2deg(rotation.getAngle()) << thrustVector.x() << thrustVector.y() << thrustVector.z();

    return thrustVector;
}



double Engine::calculateThrust(const int rpm) const
{
    // This method calculates the thrust caused by rotating the propeller at @rpm in newton.
    // As the engine/rotor-combination doesn't need to be mounted pointing straight up,
    // we return a vector of thrust, depending on @rpm and @mPose.

    Q_ASSERT(mCurrentPropeller.c1 != 0 && mCurrentPropeller.c2 != 0 && mCurrentPropeller.c3 != 0);
//    Q_ASSERT(rpm > mCurrentPropeller.rpmMin && rpm < mCurrentPropeller.rpmMax);

    double thrust = (mCurrentPropeller.c3 * pow(rpm, 2) + mCurrentPropeller.c2 * abs(rpm) + mCurrentPropeller.c1) / (1000.0 / 9.81);
    if(rpm < 0) thrust *= -1.0; // invert the thrust if we rotate backwards

    return thrust;
}

btVector3 Engine::calculateTorque(const int rpm) const
{
    // This method returns a vector of torque caused by
    Q_ASSERT(mCurrentPropeller.c1 != 0 && mCurrentPropeller.c2 != 0 && mCurrentPropeller.c3 != 0);
//    Q_ASSERT(rpm > mCurrentPropeller.rpmMin && rpm < mCurrentPropeller.rpmMax);

    static float torqueCoefficient = 0.000228 * 50000; // FIXME: this factor is made-up
    static float densityAir = 1.184;
    const float rotorDiscArea = M_PI * pow(mCurrentPropeller.diameter/2.0, 2);
    const float radiusPow3 = pow(mCurrentPropeller.diameter/2.0, 3);
    const float pitch = mCurrentPropeller.pitch; // FIXME: is this e3?
    const float torque = torqueCoefficient * densityAir * rotorDiscArea * radiusPow3 * rpm * pitch;

//    qDebug() << "Engine::calculateTorque(): rpm" << rpm << "torque" << torque;
    return btVector3(0, torque, 0);
}
*/

double Engine::calculateTorque(const int rpm) const
{
    // This method returns a vector of torque caused by
    Q_ASSERT(mCurrentPropeller.c1 != 0 && mCurrentPropeller.c2 != 0 && mCurrentPropeller.c3 != 0);
//    Q_ASSERT(rpm > mCurrentPropeller.rpmMin && rpm < mCurrentPropeller.rpmMax);

    static float torqueCoefficient = 0.228 * 10; // FIXME: this factor is made-up
    static float densityAir = 1.184;
    const float rotorDiscArea = M_PI * pow(mCurrentPropeller.diameter/2.0, 2);
    const float radiusPow3 = pow(mCurrentPropeller.diameter/2.0, 3);
    const float pitch = mCurrentPropeller.pitch; // FIXME: is this e3?
    const double torque = torqueCoefficient * densityAir * rotorDiscArea * radiusPow3 * rpm * pitch;

//    qDebug() << "Engine::calculateTorque(): rpm" << rpm << "torque" << torque;
    return torque;
}

double Engine::calculateThrust(const float& current)
{
    // Have a look at Strom-Schub.ods to find the base for these calculations
    // We also accept negative values, yielding the same thrust, just negative :)
    if(current >= 0.0)
        return pow(current*1000.0, 0.725)/85.0;
    else
        return -(pow(-current*1000.0, 0.725)/85.0);
}

btVector3 Engine::getPosition(void) const
{
    return mPose.getOrigin();
}

void Engine::initializePropellers(void)
{
    // http://www.badcock.net/ThrustXL/
    // The diameter is given in inches, is converted to meters in the Propeller c'tor

    mPropellers.insert("AERO_6050", Propeller(-0.03115116, -0.005362921, 0.000002500332, 11260, 17410, 6, 5));
    mPropellers.insert("AERO_6540", Propeller(0.09004005, -0.003803596, 0.000003055015, 4500, 17080, 6.5, 4));
    mPropellers.insert("AERO_8550", Propeller(-0.2565964, -0.01051428, 0.000007818704, 6960, 10680, 8.5, 5));
    mPropellers.insert("AERO_9560", Propeller(-0.04462973, -0.001244227, 0.00001086221, 4260, 5940, 9.5, 6));
    mPropellers.insert("APC_SF_7040", Propeller(9.505077, -0.00758246, 0.000004426797, 5020, 13650, 7, 4));
    mPropellers.insert("APC_SF_7050", Propeller(4.173811, -0.004427092, 0.000004458821, 4890, 13060, 7, 5));
    mPropellers.insert("APC_SF_7060", Propeller(4.729303, -0.003226378, 0.000004507303, 4700, 10880, 7, 6));
    mPropellers.insert("APC_SF_8038", Propeller(7.545705, -0.01175359, 0.000008335289, 4650, 10670, 8, 3.8));
    mPropellers.insert("APC_SF_8043", Propeller(5.845983, -0.00922938, 0.000008073996, 4650, 10670, 8, 4.3));
    mPropellers.insert("APC_SF_8060", Propeller(-0.8837921, -0.006203243, 0.00001025141, 4190, 8280, 8, 6));
    mPropellers.insert("APC_SF_9038", Propeller(16.1657, -0.02770201, 0.000013723, 4320, 9520, 9, 3.8));
    mPropellers.insert("APC_SF_9047", Propeller(12.62885, -0.01046025, 0.00001164577, 4160, 8590, 9, 4.7));
    mPropellers.insert("APC_SF_9060", Propeller(6.350525, -0.01850455, 0.00001778154, 3760, 7260, 9, 6));
    mPropellers.insert("APC_SF_9075", Propeller(0.8874339, -0.02086166, 0.00001947172, 4200, 6640, 9, 7.5));
    mPropellers.insert("APC_SF_1038", Propeller(2.849586, -0.05184909, 0.00002478117, 4490, 7480, 10, 3.8));
    mPropellers.insert("APC_SF_1047", Propeller(7.617356, -0.01831207, 0.00002058443, 3680, 7000, 10, 4.7));
    // Like above, just patched to allow higher RPM range
    mPropellers.insert("APC_SF_1047", Propeller(7.617356, -0.01831207, 0.00002058443, 0, 13000, 10, 4.7));
    mPropellers.insert("APC_SF_1070", Propeller(1.733572, -0.00747006, 0.00002465333, 3220, 5540, 10, 7));
    mPropellers.insert("APC_SF_1138", Propeller(1.01845, -0.0364687, 0.0000268147, 4300, 6890, 11, 3.8));
    mPropellers.insert("APC_SF_1147", Propeller(4.714039, -0.04695355, 0.00003391365, 3660, 6340, 11, 4.7));
    mPropellers.insert("APC_SF_1170", Propeller(-0.06910322, -0.021811, 0.00003867792, 3630, 4930, 11, 7));
    mPropellers.insert("APC_SF_1238", Propeller(0.1473733, -0.05656339, 0.00004107455, 3990, 5430, 12, 3.8));
    mPropellers.insert("APC_SF_1260", Propeller(-0.001613238, -0.05663766, 0.00005640653, 3510, 4120, 12, 6));
    mPropellers.insert("APC_TE_4541", Propeller(-2.801912, -0.0002272392, 0.0000007827379, 12600, 25740, 4.5, 4.1));
    mPropellers.insert("APC_TE_4742", Propeller(-1.299298, -0.0000220003, 0.0000008586717, 12180, 24600, 4.7, 4.2));
    mPropellers.insert("APC_TE_475475", Propeller(-0.7950894, -0.0003143168, 0.000000890083, 11450, 22780, 4.75, 4.75));
    mPropellers.insert("APC_TE_5050", Propeller(-0.04318849, -0.002000975, 0.000001131457, 11040, 22140, 5, 5));
    mPropellers.insert("APC_TE_525475", Propeller(0.2665206, -0.001430843, 0.000001360009, 11010, 21580, 5.25, 4.75));
    mPropellers.insert("APC_TE_5545", Propeller(-2.452231, -0.002363817, 0.000001629667, 10560, 21010, 5.5, 4.5));
    mPropellers.insert("APC_TE_6040", Propeller(4.092834, -0.00383993, 0.000002207087, 10210, 19650, 6, 4));
    mPropellers.insert("APC_TE_6529", Propeller(-0.296658, -0.002763415, 0.000002012226, 12020, 18990, 6.5, 2.9));
    mPropellers.insert("APC_TE_6537", Propeller(311.223, 0.005032671, 0.0000002509921, 1809, 15560, 6.5, 3.7));
    mPropellers.insert("APC_TE_6555", Propeller(0.1046252, -0.004411735, 0.000002842564, 10550, 14380, 6.5, 5.5));
    mPropellers.insert("APC_TE_7050", Propeller(6.141979, -0.01002906, 0.000005094205, 6180, 13830, 7, 5));
    mPropellers.insert("APC_TE_8040", Propeller(-2.266903, -0.0004225358, 0.000006179434, 4990, 12570, 8, 4));
    mPropellers.insert("APC_TE_8060", Propeller(-4.228958, -0.001256162, 0.000007684961, 4760, 9890, 8, 6));
    mPropellers.insert("APC_TE_9045", Propeller(2.700596, -0.007392398, 0.00001044568, 4780, 9970, 9, 4.5));
    mPropellers.insert("APC_TE_9060", Propeller(1.781949, -0.001381386, 0.00001155294, 4260, 8210, 9, 6));
    mPropellers.insert("APC_TE_9075", Propeller(-0.4736983, -0.003278879, 0.0000119517, 4300, 7060, 9, 7.5));
    mPropellers.insert("APC_TE_1050", Propeller(-0.2899371, -0.003940858, 0.00001432622, 4540, 8020, 10, 5));
    mPropellers.insert("APC_TE_1070", Propeller(-0.6702328, -0.003117241, 0.00001542171, 4230, 7000, 10, 7));
    mPropellers.insert("APC_TE_1155", Propeller(2.949941, -0.02309873, 0.00002248856, 4280, 7153, 11, 5.5));
    mPropellers.insert("APC_TE_1260", Propeller(0.3267369, -0.02446442, 0.00003158406, 4110, 5600, 12, 6));
    mPropellers.insert("GRP_FLD_6030", Propeller(-0.03003361, -0.006511019, 0.00000264346, 10490, 16410, 6, 3));
    mPropellers.insert("GRP_FLD_7540", Propeller(-0.03693208, -0.0008894883, 0.000004205313, 9500, 11270, 7.5, 4));
    mPropellers.insert("GRP_SPD_4747", Propeller(-0.5497109, -0.0005990636, 0.0000009109145, 12840, 23010, 4.7, 4.7));
    mPropellers.insert("GRP_SPD_5252", Propeller(-2.064986, -0.0005718618, 0.000001371636, 10250, 20830, 5.2, 5.2));
    mPropellers.insert("GRP_SPD_5543", Propeller(0.073325, -0.00260635, 0.000001736971, 9800, 21510, 5.5, 4.3));
    mPropellers.insert("GRP_SPD_5555", Propeller(-0.521598, -0.001463238, 0.000001717666, 9630, 17870, 5.5, 5.5));
    mPropellers.insert("GRP_SPD_6055", Propeller(-0.01885619, -0.0006018999, 0.000002198648, 11130, 16960, 6, 5.5));
    mPropellers.insert("GRP_SPD_6060", Propeller(-0.1533476, -0.002913859, 0.000002212202, 9280, 16730, 6, 6));
    mPropellers.insert("GRP_SPD_6565", Propeller(0.01578551, -0.0001788192, 0.000002684941, 10150, 13390, 6.5, 6.5));
    mPropellers.insert("GRP_SPD_7070", Propeller(-0.4166419, -0.005474882, 0.000003939576, 8420, 12560, 7, 7));
    mPropellers.insert("GUNTER_4943", Propeller(7.328581, -0.005968228, 0.000001791875, 8100, 21910, 4.9, 4.3));
    mPropellers.insert("GWS_HD_4540", Propeller(-0.218308, 0.0000388775, 0.0000006705584, 5800, 17200, 4.5, 4));
    mPropellers.insert("GWS_HD_5043", Propeller(0.5451553, 0.0006530647, 0.0000008536912, 4600, 14860, 5, 4.3));
    mPropellers.insert("GWS_HD_6030", Propeller(-1.511471, -0.001593753, 0.000001700649, 6360, 15210, 6, 3));
    mPropellers.insert("GWS_HD_7035", Propeller(11.26809, -0.009156602, 0.000003051149, 6840, 14190, 7, 3.5));
    mPropellers.insert("GWS_HD_8040", Propeller(0.4362173, -0.002892073, 0.000005363623, 4950, 10890, 8, 4));
    mPropellers.insert("GWS_HD_8060", Propeller(-0.8400515, -0.0009485304, 0.000007012877, 5400, 9690, 8, 6));
    mPropellers.insert("GWS_HD_9050", Propeller(5.11956, -0.01010855, 0.00001052979, 4120, 9480, 9, 5));
    mPropellers.insert("GWS_HD_1060", Propeller(-0.6654808, -0.001428664, 0.00001297712, 3960, 7260, 10, 6));
    mPropellers.insert("GWS_HD_1080", Propeller(0.09679207, -0.003395906, 0.00001735257, 3540, 4620, 10, 8));
    mPropellers.insert("GWS_HD_1170", Propeller(-15.03829, 0.003385303, 0.00002100382, 2310, 6750, 11, 7));
    mPropellers.insert("GWS_HD_1280", Propeller(4.693693, -0.02930364, 0.00003718949, 2250, 5100, 12, 8));
    mPropellers.insert("GWS_RS_6050", Propeller(0.835661, -0.002490498, 0.000002894121, 4050, 10530, 6, 5));
    mPropellers.insert("GWS_RS_7060", Propeller(-1.632629, 0.0007775814, 0.000005252213, 2760, 9960, 7, 6));
    mPropellers.insert("GWS_RS_8043", Propeller(-2.755759, 0.002637239, 0.000006219468, 2970, 10050, 8, 4.3));
    mPropellers.insert("GWS_RS_8060", Propeller(1.051953, -0.003372149, 0.000008401962, 2730, 8040, 8, 6));
    mPropellers.insert("GWS_RS_9047", Propeller(-2.833769, 0.0004081182, 0.00001093588, 2490, 8280, 9, 4.7));
    mPropellers.insert("GWS_RS_9070", Propeller(1.910723, -0.006790855, 0.00001475528, 2190, 6180, 9, 7));
    mPropellers.insert("GWS_RS_9070_3", Propeller(1.55224, -0.005030068, 0.00001490447, 2260, 6360, 9, 7));
    mPropellers.insert("GWS_RS_1047", Propeller(-0.18737, -0.008525119, 0.00001884232, 2190, 7380, 10, 4.7));
    mPropellers.insert("GWS_RS_1080", Propeller(-1.552253, -0.011761, 0.00002707483, 2220, 4620, 10, 8));
    mPropellers.insert("GWS_RS_1147", Propeller(9.348635, -0.02296219, 0.00002899345, 1440, 6390, 11, 4.7));
    mPropellers.insert("GWS_RS_1180", Propeller(-0.5706088, -0.004354384, 0.00003228519, 1800, 4890, 11, 8));
    mPropellers.insert("GWS_RS_1260", Propeller(1.348293, -0.006029384, 0.00003430268, 1470, 5130, 12, 6));
    mPropellers.insert("GWS_RS_1280", Propeller(3.922396, -0.01364813, 0.00004710823, 1320, 4260, 12, 8));
    mPropellers.insert("GWS_RS_1365", Propeller(1.600268, -0.01334862, 0.00005120779, 1290, 4260, 13, 6.5));
    mPropellers.insert("GWS_RS_1390", Propeller(5.057864, -0.02439314, 0.00006687729, 1290, 3780, 13, 9));
    mPropellers.insert("GWS_RS_1470", Propeller(4.903353, -0.03685949, 0.00007014424, 1470, 4020, 14, 7));
    mPropellers.insert("IKARUS_9060", Propeller(0.06924966, -0.002321077, 0.00001319245, 4080, 5610, 9, 6));
}
