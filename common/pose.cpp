#include "pose.h"

    typedef struct {float x, y, z, w;} Quat; /* Quaternion */
    enum QuatPart {X, Y, Z, W};
    typedef qreal HMatrix[4][4]; /* Right-handed, for column vectors */
    typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */

    /*** Order type constants, constructors, extractors ***/
    /* There are 24 possible conventions, designated by:    */
    /*	  o EulAxI = axis used initially		    */
    /*	  o EulPar = parity of axis permutation		    */
    /*	  o EulRep = repetition of initial axis as last	    */
    /*	  o EulFrm = frame from which axes are taken	    */
    /* Axes I,J,K will be a permutation of X,Y,Z.	    */
    /* Axis H will be either I or K, depending on EulRep.   */
    /* Frame S takes axes from initial static frame.	    */
    /* If ord = (AxI=X, Par=Even, Rep=No, Frm=S), then	    */
    /* {a,b,c,ord} means Rz(c)Ry(b)Rx(a), where Rz(c)v	    */
    /* rotates v around Z by c radians.			    */
#define EulFrmS	     0
#define EulFrmR	     1
#define EulFrm(ord)  ((unsigned)(ord)&1)
#define EulRepNo     0
#define EulRepYes    1
#define EulRep(ord)  (((unsigned)(ord)>>1)&1)
#define EulParEven   0
#define EulParOdd    1
#define EulPar(ord)  (((unsigned)(ord)>>2)&1)
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
#define EulAxI(ord)  ((int)(EulSafe[(((unsigned)(ord)>>3)&3)]))
#define EulAxJ(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)==EulParOdd)]))
#define EulAxK(ord)  ((int)(EulNext[EulAxI(ord)+(EulPar(ord)!=EulParOdd)]))
#define EulAxH(ord)  ((EulRep(ord)==EulRepNo)?EulAxK(ord):EulAxI(ord))
    /* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
    /* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
    /* Static axes */
#define EulOrdXYZs    EulOrd(X,EulParEven,EulRepNo,EulFrmS)
#define EulOrdXYXs    EulOrd(X,EulParEven,EulRepYes,EulFrmS)
#define EulOrdXZYs    EulOrd(X,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdXZXs    EulOrd(X,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdYZXs    EulOrd(Y,EulParEven,EulRepNo,EulFrmS)
#define EulOrdYZYs    EulOrd(Y,EulParEven,EulRepYes,EulFrmS)
#define EulOrdYXZs    EulOrd(Y,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdYXYs    EulOrd(Y,EulParOdd,EulRepYes,EulFrmS)
#define EulOrdZXYs    EulOrd(Z,EulParEven,EulRepNo,EulFrmS)
#define EulOrdZXZs    EulOrd(Z,EulParEven,EulRepYes,EulFrmS)
#define EulOrdZYXs    EulOrd(Z,EulParOdd,EulRepNo,EulFrmS)
#define EulOrdZYZs    EulOrd(Z,EulParOdd,EulRepYes,EulFrmS)
    /* Rotating axes */
#define EulOrdZYXr    EulOrd(X,EulParEven,EulRepNo,EulFrmR)
#define EulOrdXYXr    EulOrd(X,EulParEven,EulRepYes,EulFrmR)
#define EulOrdYZXr    EulOrd(X,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdXZXr    EulOrd(X,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdXZYr    EulOrd(Y,EulParEven,EulRepNo,EulFrmR)
#define EulOrdYZYr    EulOrd(Y,EulParEven,EulRepYes,EulFrmR)
#define EulOrdZXYr    EulOrd(Y,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdYXYr    EulOrd(Y,EulParOdd,EulRepYes,EulFrmR)
#define EulOrdYXZr    EulOrd(Z,EulParEven,EulRepNo,EulFrmR)
#define EulOrdZXZr    EulOrd(Z,EulParEven,EulRepYes,EulFrmR)
#define EulOrdXYZr    EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
#define EulOrdZYZr    EulOrd(Z,EulParOdd,EulRepYes,EulFrmR)

    EulerAngles Eul_(float ai, float aj, float ah, int order);
    Quat Eul_ToQuat(EulerAngles ea);
    void Eul_ToHMatrix(EulerAngles ea, HMatrix M);
    EulerAngles Eul_FromHMatrix(HMatrix M, int order);
    EulerAngles Eul_FromQuat(Quat q, int order);



    EulerAngles Eul_(float ai, float aj, float ah, int order)
    {
        EulerAngles ea;
        ea.x = ai; ea.y = aj; ea.z = ah;
        ea.w = order;
        return (ea);
    }
    /* Construct quaternion from Euler angles (in radians). */
    Quat Eul_ToQuat(EulerAngles ea)
    {
        Quat qu;
        double a[3], ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
        int i,j,k,h,n,s,f;
        EulGetOrd(ea.w,i,j,k,h,n,s,f);
        if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
        if (n==EulParOdd) ea.y = -ea.y;
        ti = ea.x*0.5; tj = ea.y*0.5; th = ea.z*0.5;
        ci = cos(ti);  cj = cos(tj);  ch = cos(th);
        si = sin(ti);  sj = sin(tj);  sh = sin(th);
        cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
        if (s==EulRepYes) {
            a[i] = cj*(cs + sc);	/* Could speed up with */
            a[j] = sj*(cc + ss);	/* trig identities. */
            a[k] = sj*(cs - sc);
            qu.w = cj*(cc - ss);
        } else {
            a[i] = cj*sc - sj*cs;
            a[j] = cj*ss + sj*cc;
            a[k] = cj*cs - sj*sc;
            qu.w = cj*cc + sj*ss;
        }
        if (n==EulParOdd) a[j] = -a[j];
        qu.x = a[X]; qu.y = a[Y]; qu.z = a[Z];
        return (qu);
    }

    /* Construct matrix from Euler angles (in radians). */
    void Eul_ToHMatrix(EulerAngles ea, HMatrix M)
    {
        double ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
        int i,j,k,h,n,s,f;
        EulGetOrd(ea.w,i,j,k,h,n,s,f);
        if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
        if (n==EulParOdd) {ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z;}
        ti = ea.x;	  tj = ea.y;	th = ea.z;
        ci = cos(ti); cj = cos(tj); ch = cos(th);
        si = sin(ti); sj = sin(tj); sh = sin(th);
        cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
        if (s==EulRepYes) {
            M[i][i] = cj;	  M[i][j] =  sj*si;    M[i][k] =  sj*ci;
            M[j][i] = sj*sh;  M[j][j] = -cj*ss+cc; M[j][k] = -cj*cs-sc;
            M[k][i] = -sj*ch; M[k][j] =  cj*sc+cs; M[k][k] =  cj*cc-ss;
        } else {
            M[i][i] = cj*ch; M[i][j] = sj*sc-cs; M[i][k] = sj*cc+ss;
            M[j][i] = cj*sh; M[j][j] = sj*ss+cc; M[j][k] = sj*cs-sc;
            M[k][i] = -sj;	 M[k][j] = cj*si;    M[k][k] = cj*ci;
        }
        M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0; M[W][W]=1.0;
    }

    /* Convert matrix to Euler angles (in radians). */
    EulerAngles Eul_FromHMatrix(HMatrix M, int order)
    {
        EulerAngles ea;
        int i,j,k,h,n,s,f;
        EulGetOrd(order,i,j,k,h,n,s,f);
        if (s==EulRepYes) {
            double sy = sqrt(M[i][j]*M[i][j] + M[i][k]*M[i][k]);
            if (sy > 16*FLT_EPSILON) {
                ea.x = atan2(M[i][j], M[i][k]);
                ea.y = atan2(sy, M[i][i]);
                ea.z = atan2(M[j][i], -M[k][i]);
            } else {
                ea.x = atan2(-M[j][k], M[j][j]);
                ea.y = atan2(sy, M[i][i]);
                ea.z = 0;
            }
        } else {
            double cy = sqrt(M[i][i]*M[i][i] + M[j][i]*M[j][i]);
            if (cy > 16*FLT_EPSILON) {
                ea.x = atan2(M[k][j], M[k][k]);
                ea.y = atan2(-M[k][i], cy);
                ea.z = atan2(M[j][i], M[i][i]);
            } else {
                ea.x = atan2(-M[j][k], M[j][j]);
                ea.y = atan2(-M[k][i], cy);
                ea.z = 0;
            }
        }
        if (n==EulParOdd) {ea.x = -ea.x; ea.y = - ea.y; ea.z = -ea.z;}
        if (f==EulFrmR) {float t = ea.x; ea.x = ea.z; ea.z = t;}
        ea.w = order;
        return (ea);
    }

    /* Convert quaternion to Euler angles (in radians). */
    EulerAngles Eul_FromQuat(Quat q, int order)
    {
        HMatrix M;
        double Nq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
        double xs = q.x*s,	  ys = q.y*s,	 zs = q.z*s;
        double wx = q.w*xs,	  wy = q.w*ys,	 wz = q.w*zs;
        double xx = q.x*xs,	  xy = q.x*ys,	 xz = q.x*zs;
        double yy = q.y*ys,	  yz = q.y*zs,	 zz = q.z*zs;
        M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
        M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
        M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
        M[W][X]=M[W][Y]=M[W][Z]=M[X][W]=M[Y][W]=M[Z][W]=0.0; M[W][W]=1.0;
        return (Eul_FromHMatrix(M, order));
    }






Pose::Pose()
{
    mTransform.setToIdentity();
    precision = 0;
    covariances = 100.0f;
    this->timestamp = 0;
}

Pose::Pose(const QMatrix4x4& matrix, const qint32& timestamp)
{
    mTransform = matrix;

    if(timestamp == 0)
        this->timestamp = getCurrentGpsTowTime();
    else
        this->timestamp = timestamp;
}
/*
Pose::Pose(const QVector3D &position, const QQuaternion &orientation, const qint32& timestamp)
{
    const float yaw = keepWithinRangeRadians(getYawRadians(orientation, true));
    const float pitch = keepWithinRangeRadians(getPitchRadians(orientation, true));
    const float roll = keepWithinRangeRadians(getRollRadians(orientation, true));

    mTransform.setToIdentity();
    mTransform.translate(position);
    mTransform.rotate(yaw, QVector3D(0,1,0));
    mTransform.rotate(pitch, QVector3D(1,0,0));
    mTransform.rotate(roll, QVector3D(0,0,1));

    if(timestamp == 0)
        this->timestamp = getCurrentGpsTowTime();
    else
        this->timestamp = timestamp;

    precision = 0;
    covariances = 100.0f;
}*/

Pose::Pose(const QVector3D &position, const float &yawDegrees, const float &pitchDegrees, const float &rollDegrees, const qint32& timestamp)
{
    mTransform.setToIdentity();
    mTransform.translate(position);
    mTransform.rotate(yawDegrees, QVector3D(0,1,0));
    mTransform.rotate(pitchDegrees, QVector3D(1,0,0));
    mTransform.rotate(rollDegrees, QVector3D(0,0,1));

    precision = 0;
    covariances = 100.0f;

    if(timestamp == 0)
        this->timestamp = getCurrentGpsTowTime();
    else
        this->timestamp = timestamp;
}

const QVector3D Pose::getPosition() const
{
    const QVector4D unitVector = QVector4D(0.0f, 0.0f, 0.0f, 1.0f);
    return mTransform.map(unitVector).toVector3D();
}

void Pose::getEulerAnglesDegrees(float& yaw, float &pitch, float &roll) const
{
    getEulerAnglesRadians(yaw, pitch, roll);
    yaw = RAD2DEG(yaw);
    pitch = RAD2DEG(pitch);
    roll = RAD2DEG(roll);
}

void Pose::getEulerAnglesRadians(float& yaw, float &pitch, float &roll) const
{
    EulerAngles outAngs;
    qreal matrixElements[4][4];
    memcpy(&matrixElements, mTransform.data(), sizeof(qreal)*16);
    // We rotate yaw, pitch, roll, which is Y-X-Z, which seems to need being specified backwards
    // The R means that our axes rotate instead of being static during rotation
    outAngs = Eul_FromHMatrix(matrixElements, EulOrdZXYr);
    yaw = -outAngs.z;
    pitch = -outAngs.y;
    roll = -outAngs.x;
}

/*
  **************************************************************************
  ************************ interpolation methods ***************************
  **************************************************************************
  */

Pose Pose::interpolateLinear(const Pose &before, const Pose &after, const float &mu)
{
    Q_ASSERT(mu <= 0.0 && mu <= 1.0);

    float beforeYaw, beforePitch, beforeRoll;
    float afterYaw, afterPitch, afterRoll;

    before.getEulerAnglesDegrees(beforeYaw, beforePitch, beforeRoll);
    after.getEulerAnglesDegrees(afterYaw, afterPitch, afterRoll);

    Pose p(
                before.getPosition() * (1.0 - mu) + after.getPosition() * mu,
                beforeYaw * (1.0 - mu) + afterYaw * mu,
                beforePitch * (1.0 - mu) + afterPitch * mu,
                beforeRoll * (1.0 - mu) + afterRoll * mu,
                before.timestamp * (1.0 - mu) + after.timestamp * mu
                );

    p.covariances = before.covariances * (1.0 - mu) + after.covariances * mu;
    p.precision = before.precision & after.precision;

    return p;
}

// http://paulbourke.net/miscellaneous/interpolation/

// TODO: This code doesn't really use the timestamps in the poses, that seems stupid
Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const float &mu)
{//                                             y0                        y1                         y2                        y3
    Q_ASSERT(mu >= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
    const QVector3D po0 = last->getPosition() - after->getPosition() - first->getPosition() + before->getPosition();
    const QVector3D po1 = first->getPosition() - before->getPosition() - po0;
    const QVector3D po2 = after->getPosition() - first->getPosition();
    const QVector3D po3 = before->getPosition();

    QVector3D resultPosition = po0*mu*mu2+po1*mu2+po2*mu+po3;

    float firstYaw, firstPitch, firstRoll;
    first->getEulerAnglesRadians(firstYaw, firstPitch, firstRoll);

    float beforeYaw, beforePitch, beforeRoll;
    before->getEulerAnglesRadians(beforeYaw, beforePitch, beforeRoll);

    float afterYaw, afterPitch, afterRoll;
    after->getEulerAnglesRadians(afterYaw, afterPitch, afterRoll);

    float lastYaw, lastPitch, lastRoll;
    last->getEulerAnglesRadians(lastYaw, lastPitch, lastRoll);

    // yaw - what happens with values around +-180?!r
    const float y0 = lastYaw - afterYaw - firstYaw + beforeYaw;
    const float y1 = firstYaw - beforeYaw - y0;
    const float y2 = afterYaw - firstYaw;
    const float y3 = beforeYaw;

    const float yaw = y0*mu*mu2+y1*mu2+y2*mu+y3;

    // pitch
    const float p0 = lastPitch - afterPitch - firstPitch + beforePitch;
    const float p1 = firstPitch - beforePitch - p0;
    const float p2 = afterPitch - firstPitch;
    const float p3 = beforePitch;

    const float pitch = p0*mu*mu2+p1*mu2+p2*mu+p3;

    // roll
    const float r0 = lastRoll - afterRoll - firstRoll + beforeRoll;
    const float r1 = firstRoll - beforeRoll - r0;
    const float r2 = afterRoll - firstRoll;
    const float r3 = beforeRoll;

    const float roll = r0*mu*mu2+r1*mu2+r2*mu+r3;

    // time
    const float t0 = last->timestamp - after->timestamp - first->timestamp + before->timestamp;
    const float t1 = first->timestamp - before->timestamp - t0;
    const float t2 = after->timestamp - first->timestamp;
    const float t3 = before->timestamp;

    const qint32 timestamp = t0*mu*mu2+t1*mu2+t2*mu+t3;

    Pose p(resultPosition, yaw, pitch, roll, timestamp);
    p.covariances = (before->covariances + after->covariances) / 2.0f;
    p.precision = before->precision & after->precision;

    return p;
}

Pose Pose::interpolateCubic(const Pose * const first, const Pose * const before, const Pose * const after, const Pose * const last, const qint32& time)
{//                                             y0                        y1                         y2                        y3

    // Check parameters.
    Q_ASSERT(first->timestamp < before->timestamp && "Pose::interpolateCubic(): first < before didn't pass");

    if(!(before->timestamp <= time)) qDebug() << "Pose::interpolateCubic(): before" << before->timestamp << "<= raytime" << time <<  "didn't pass";
    if(!(after->timestamp >= time)) qDebug() << "Pose::interpolateCubic(): after" << after->timestamp << ">= raytime" << time <<  "didn't pass";

    Q_ASSERT(last->timestamp > after->timestamp && "Pose::interpolateCubic(): last > after didn't pass");

    // recreate mu from time argument
    const float mu = (((float)(time - before->timestamp)) / ((float)(after->timestamp - before->timestamp)));
    Pose p = interpolateCubic(first, before, after, last, mu);
    p.timestamp = time;
    return p;
}

/*
  **************************************************************************
  ************************                       ***************************
  **************************************************************************
  */
float Pose::getShortestTurnRadians(float angle)
{
    return DEG2RAD(getShortestTurnDegrees(RAD2DEG(angle)));
}

float Pose::getShortestTurnDegrees(float angle)
{
    while(angle <= -180.0) angle += 360.0;
    while(angle > 180.0) angle -= 360.0;
    return angle;
}

QVector2D Pose::getPlanarPosition() const
{
    const QVector3D position = getPosition();
    return QVector2D(position.x(), position.z());
}

// No idea whether the order of orientation is correct
const QQuaternion Pose::getOrientation() const
{
    // FIXME: This is buggy! It was used for buggy laserscanner-fusion, too.
    return
            QQuaternion::fromAxisAndAngle(QVector3D(0,1,0), getYawDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), getPitchDegrees())
            * QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), getRollDegrees());
}

QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << pose.toString();

    return dbg.space();
}

const QString Pose::toString() const
{
    const QVector3D position = getPosition();

    return QString()
            .append("pose t").append(QString::number(timestamp))
            .append(" (").append(QString::number(position.x(), 'f', 2))
            .append("/").append(QString::number(position.y(), 'f', 2))
            .append("/").append(QString::number(position.z(), 'f', 2))
            .append(") YPR (").append(QString::number(getYawDegrees(), 'f', 2))
            .append("/").append(QString::number(getPitchDegrees(), 'f', 2))
            .append("/").append(QString::number(getRollDegrees(), 'f', 2)).append(")")
            .append(" pr ").append(QString::number(precision))
            .append(" co ").append(QString::number(covariances, 'f', 2));
}

const QString Pose::getFlagsString() const
{
    QString flags;
    if(precision & Pose::AttitudeAvailable) flags.append("+ATT"); else flags.append("-ATT");
    if(precision & Pose::CorrectionAgeLow) flags.append(" +COR"); else flags.append(" -COR");
    if(precision & Pose::RtkFixed) flags.append(" +RTK"); else flags.append(" -RTK");
    if(precision & Pose::ModeIntegrated) flags.append(" +INT"); else flags.append(" -INT");
    return flags;
}

const QString Pose::toStringVerbose() const
{
    const QVector3D position = getPosition();

    return QString()
            .append("pose t").append(QString::number(timestamp))
            .append(" (").append(QString::number(position.x(), 'f', 2))
            .append("/").append(QString::number(position.y(), 'f', 2))
            .append("/").append(QString::number(position.z(), 'f', 2))
            .append(") YPR (").append(QString::number(getYawDegrees(), 'f', 2))
            .append("/").append(QString::number(getPitchDegrees(), 'f', 2))
            .append("/").append(QString::number(getRollDegrees(), 'f', 2)).append(")")
            .append(" pr ").append(getFlagsString())
            .append(" co ").append(QString::number(covariances, 'f', 2));
}

// Must be able to process what operator<< writes above, for example:
// pose t501171350 (-30.49/51.84/140.01) YPR (155.27/2.92/-1.03) pr 17 co 2.34
Pose::Pose(const QString& poseString)
{
    QStringList tokens = poseString.split(' ');
    if(tokens.size() != 9) qDebug() << "Pose::Pose(QString): token stringlist size is not 5!";
    bool success = false;

    // set time
    const qint32 timestamp = tokens.value(1).remove(0, 1).toInt(&success, 10);
    Q_ASSERT(success && "Pose::Pose(QString): couldn't convert time to int.");
    this->timestamp = timestamp;

    mTransform.setToIdentity();

    // set positions
    QVector3D position;
    QString positionString = tokens.value(2).remove(0, 1);
    positionString.chop(1);
    const QStringList positions = positionString.split('/');
    position.setX(positions.at(0).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert X to float.";
    position.setY(positions.at(1).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert Y to float.";
    position.setZ(positions.at(2).toFloat(&success));
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert Z to float.";

    mTransform.translate(position);

    // set orientations
    float yaw, pitch, roll;
    QString orientationString = tokens.value(4).remove(0, 1);
    orientationString.chop(1);
    const QStringList orientations = orientationString.split('/');
    yaw = orientations.at(0).toFloat(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert yaw to float.";
    pitch = orientations.at(1).toFloat(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert pitch to float.";
    roll = orientations.at(2).toFloat(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert roll to float.";

    mTransform.rotate(yaw, QVector3D(0,1,0));
    mTransform.rotate(pitch, QVector3D(1,0,0));
    mTransform.rotate(roll, QVector3D(0,0,1));

    // set precision and covariances
    precision = tokens.value(6).toInt(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert precision to int.";

    covariances = tokens.value(8).toFloat(&success);
    if(!success) qDebug() << "Pose::Pose(QString): couldn't convert covariances to float.";

    if(poseString != toString())
        qDebug() << "Pose::Pose(QString): parsing failed: original:" << poseString << "reconstructed" << toString();
}

QDataStream& operator<<(QDataStream &out, const Pose &pose)
{
    out << pose.getMatrix();
    out << pose.timestamp;
    out << pose.precision;
    out << pose.covariances;
    return out;
}

QDataStream& operator>>(QDataStream &in, Pose &pose)
{
    QMatrix4x4 matrix;
    in >> matrix;
    in >> pose.timestamp;
    in >> pose.precision;
    in >> pose.covariances;

    pose.setMatrix(matrix);

    return in;
}
/*
float Pose::getRollRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // ben: this was Real in ogre, so it might be better to use double
        // roll = atan2(localx.y, localx.x)
        // pick parts of xAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwz = fTz * orientation.scalar();
        float fTxy = fTy * orientation.x();
        float fTyy = fTy * orientation.y();
        float fTzz = fTz * orientation.z();

        // Vector3(1.0-(fTyy+fTzz), fTxy+fTwz, fTxz-fTwy);
        return atan2(fTxy+fTwz, 1.0-(fTyy+fTzz));
    }
    else
    {
        return atan2(2*(orientation.x() * orientation.y() + orientation.scalar() * orientation.z() ), orientation.scalar() * orientation.scalar() + orientation.x() * orientation.x() - orientation.y() * orientation.y() - orientation.z() * orientation.z());
    }
}

float Pose::getPitchRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // pitch = atan2(localy.z, localy.y)
        // pick parts of yAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwx = fTx * orientation.scalar();
        float fTxx = fTx * orientation.x();
        float fTyz = fTz * orientation.y();
        float fTzz = fTz * orientation.z();

        // Vector3(fTxy-fTwz, 1.0-(fTxx+fTzz), fTyz+fTwx);
        return atan2(fTyz+fTwx, 1.0-(fTxx+fTzz));
    }
    else
    {
        // internal version
        return atan2(2*(orientation.y() * orientation.z() + orientation.scalar() * orientation.x() ), orientation.scalar() * orientation.scalar() - orientation.x() * orientation.x() - orientation.y() * orientation.y() + orientation.z() * orientation.z());
    }
}

float Pose::getYawRadians(const QQuaternion& orientation, bool reprojectAxis)
{
    if(reprojectAxis)
    {
        // yaw = atan2(localz.x, localz.z)
        // pick parts of zAxis() implementation that we need
        float fTx  = 2.0 * orientation.x();
        float fTy  = 2.0 * orientation.y();
        float fTz  = 2.0 * orientation.z();
        float fTwy = fTy * orientation.scalar();
        float fTxx = fTx * orientation.x();
        float fTxz = fTz * orientation.x();
        float fTyy = fTy * orientation.y();

        // Vector3(fTxz+fTwy, fTyz-fTwx, 1.0-(fTxx+fTyy));
        return atan2(fTxz+fTwy, 1.0-(fTxx+fTyy));
    }
    else
    {
        // internal version
        return asin(-2*(orientation.x() * orientation.z() - orientation.scalar() * orientation.y()));
    }
}
*/










