#include "pose.h"

    typedef struct {float x, y, z, w;} Quat; /* Quaternion */
    enum QuatPart {X, Y, Z, W};
    typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
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
    timestamp = 0;
    acceleration = 0.0f;
    rotation = 0.0f;
}

Pose::Pose(const QMatrix4x4& matrix, const qint32& timestamp)
{
    mTransform = matrix;

    if(timestamp == 0)
        this->timestamp = GnssTime::currentTow();
    else
        this->timestamp = timestamp;

    mVelocity = QVector3D();
    acceleration = 0.0f;
    rotation = 0.0f;
}

Pose::Pose(const Pose* const p)
{
    mTransform = p->getMatrixConst();
    timestamp = p->timestamp;

    mVelocity = QVector3D();
    acceleration = 0.0f;
    rotation = 0.0f;
}

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
        this->timestamp = GnssTime::currentTow();
    else
        this->timestamp = timestamp;

    mVelocity = QVector3D();
    acceleration = 0.0f;
    rotation = 0.0f;
}

const QVector3D Pose::getPosition() const
{
    // FIXME: testing!
    return mTransform.column(3).toVector3D();

//    const QVector4D unitVector = QVector4D(0.0f, 0.0f, 0.0f, 1.0f);
//    return mTransform.map(unitVector).toVector3D();
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
    float matrixElements[4][4];
    memcpy(&matrixElements, mTransform.data(), sizeof(float)*16);
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

Pose Pose::extrapolateLinear(const Pose &p1, const Pose &p2, const qint32 &timeInFuture)
{
    Q_ASSERT(p2.timestamp >= p1.timestamp);

    const QQuaternion& q1 = p1.getOrientation();
    const QQuaternion& q2 = p2.getOrientation();

    const QQuaternion delta = q2 * q1.conjugate();
    const float extrapolationFactor = (timeInFuture-p1.timestamp)/(p2.timestamp-p1.timestamp);

//    qDebug() << "ext:" << p1.timestamp << p2.timestamp << timeInFuture << extrapolationFactor;

    const QVector3D rotationAxis(
                delta.x() / sqrt(1-delta.scalar()*delta.scalar()),
                delta.y() / sqrt(1-delta.scalar()*delta.scalar()),
                delta.z() / sqrt(1-delta.scalar()*delta.scalar())
                );

    float rotationAngle = 2 * acos(delta.scalar());
    while (rotationAngle > 180.0f) rotationAngle -= 360.0f;
    rotationAngle = rotationAngle * extrapolationFactor;
    rotationAngle = fmod(rotationAngle, 360.0f);

    //    qDebug() << "ext: rotation of" << RAD2DEG(rotationAngle) << "around" << rotationAxis;

    QQuaternion q3 = QQuaternion::fromAxisAndAngle(rotationAxis, RAD2DEG(rotationAngle));

    Pose p(
                (p2.getPosition()-p1.getPosition()) * extrapolationFactor, // linear extrapolation of position
                0, 0, 0, // set rotations to zero. We'll set them later using our quaternions
                timeInFuture); // this is the time we wanted

    p.getMatrixRef().rotate(q1); // apply original rotation
    p.getMatrixRef().rotate(q3); // add extrapolated rotation

    p.covariances = p2.covariances + (p2.covariances - p1.covariances) * extrapolationFactor;
    p.precision = p1.precision & p2.precision;

    return p;
}

Pose Pose::interpolateLinear(const Pose* const p0, const Pose* const p1, const qint32& time)
{
    // recreate mu from time argument
    const float mu = (((float)(time - p0->timestamp)) / ((float)(p1->timestamp - p0->timestamp)));
    Q_ASSERT(mu >= 0.0 && mu <= 1.0);

    const QVector3D position = p0->getPosition() * (1.0 - mu) + p1->getPosition() * mu;
    const QQuaternion orientation = QQuaternion::nlerp(p0->getOrientation(), p1->getOrientation(), mu);

    QMatrix4x4 m;
    m.translate(position);
    m.rotate(orientation);

    Pose p;
    p.setMatrix(m);

    p.covariances = p0->covariances * (1.0f - mu) + p1->covariances * mu;
    p.precision = p0->precision & p1->precision; // yes, thats a logic AND
    p.timestamp = time;

    p.rotation = p0->rotation * (1.0f - mu) + p1->rotation * mu;
    p.mVelocity = p0->mVelocity * (1.0f - mu) + p1->mVelocity * mu;
    p.acceleration = p0->acceleration * (1.0f - mu) + p1->acceleration * mu;

    return p;
}

// http://paulbourke.net/miscellaneous/interpolation/
// TODO: This code doesn't really use the timestamps in the poses, that seems stupid
QVector3D Pose::interpolateCubic(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, const QVector3D& p3, const float mu)
{
    Q_ASSERT(mu >= 0.0 && mu <= 1.0);

    const double mu2 = mu*mu;

    // position
    const QVector3D pos0 = p3 - p2 - p0 + p1;
    const QVector3D pos1 = p0 - p1 - pos0;
    const QVector3D pos2 = p2 - p0;
    const QVector3D pos3 = p1;

    const QVector3D resultPosition = pos0*mu*mu2+pos1*mu2+pos2*mu+pos3;

    return resultPosition;
}

Pose Pose::interpolateCubic(const Pose * const p0, const Pose * const p1, const Pose * const p2, const Pose * const p3, const qint32& time)
{//                                             y0                        y1                         y2                        y3

    // Check parameters.
    Q_ASSERT(p0->timestamp < p1->timestamp && "Pose::interpolateCubic(): first < before didn't pass");

    if(!(p1->timestamp <= time)) qDebug() << "Pose::interpolateCubic(): p1" << p1->timestamp << "<= raytime" << time <<  "didn't pass";
    if(!(p2->timestamp >= time)) qDebug() << "Pose::interpolateCubic(): p2" << p2->timestamp << ">= raytime" << time <<  "didn't pass";

    Q_ASSERT(p3->timestamp > p2->timestamp && "Pose::interpolateCubic(): t3 > t2 didn't pass");

    // recreate mu from time argument
    const float mu = (((float)(time - p1->timestamp)) / ((float)(p2->timestamp - p1->timestamp)));

    const QVector3D position = interpolateCubic(
                p0->getPosition(),
                p1->getPosition(),
                p2->getPosition(),
                p3->getPosition(),
                mu);

    // cubic - order 1 (seems wrong, vehicle motion has "sawtooth steps"
    //const QQuaternion orientation = interpolateCubic(p0->getOrientation(), p1->getOrientation(), p2->getOrientation(), p3->getOrientation(), mu);

    // cubic - order 2 (seems right)
    const QQuaternion orientation = interpolateCubic(p1->getOrientation(), p0->getOrientation(), p3->getOrientation(), p2->getOrientation(), mu);

    // linear
    //const QQuaternion orientation = QQuaternion::slerp(p1->getOrientation(), p2->getOrientation(), mu);

    QMatrix4x4 m;
    m.translate(position);
    m.rotate(orientation);

    Pose p;
    p.setMatrix(m);

    p.covariances = (p1->covariances + p2->covariances) / 2.0f; // average
    p.precision = p1->precision & p2->precision; // yes, thats a logic AND
    p.timestamp = time;

    p.rotation = p1->rotation * (1.0f - mu) + p2->rotation * mu;
    p.mVelocity = p1->mVelocity * (1.0f - mu) + p2->mVelocity * mu;
    p.acceleration = p1->acceleration * (1.0f - mu) + p2->acceleration * mu;

    return p;
}

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

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

const QQuaternion Pose::getOrientation() const
{
    // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    float x,y,z,w;

    w = sqrt( std::max( 0.0f, 1 + mTransform(0,0) + mTransform(1,1)+ mTransform(2,2)) ) / 2;
    x = sqrt( std::max( 0.0f, 1 + mTransform(0,0) - mTransform(1,1)- mTransform(2,2)) ) / 2;
    y = sqrt( std::max( 0.0f, 1 - mTransform(0,0) + mTransform(1,1)- mTransform(2,2)) ) / 2;
    z = sqrt( std::max( 0.0f, 1 - mTransform(0,0) - mTransform(1,1)+ mTransform(2,2)) ) / 2;

    x *= sign( x * ( mTransform(2,1)- mTransform(1,2)) );
    y *= sign( y * ( mTransform(0,2)- mTransform(2,0)) );
    z *= sign( z * ( mTransform(1,0)- mTransform(0,1)) );

    return QQuaternion(w,x,y,z);
}

QQuaternion Pose::inverse(const QQuaternion& q)
{
    float fNorm = q.scalar()*q.scalar()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z();
    if(fNorm > 0.0f)
    {
        float fInvNorm = 1.0f/fNorm;
        return QQuaternion(q.scalar()*fInvNorm,-q.x()*fInvNorm,-q.y()*fInvNorm,-q.z()*fInvNorm);
    }
    else
    {
        // return an invalid result to flag the error
        qFatal("invalid quat!");
    }
}

void Pose::rotationToAngleAxis(const QQuaternion& q, float& angleDegrees, QVector3D& axis)
{
    // The quaternion representing the rotation is
    //   q = cos(A/2)+sin(A/2)*(x*i+y*j+z*k)

    float fSqrLength = q.x()*q.x()+q.y()*q.y()+q.z()*q.z();

    if(fSqrLength > 0.0f)
    {
        angleDegrees = RAD2DEG(2.0*acos(q.scalar()));
        //float fInvLength = Math::InvSqrt(fSqrLength);
        float fInvLength = 1/std::sqrt(fSqrLength);
        axis.setX(q.x()*fInvLength);
        axis.setY(q.y()*fInvLength);
        axis.setZ(q.z()*fInvLength);
    }
    else
    {
        // angle is 0 (mod 2*pi), so any axis will do
        angleDegrees = 0.0f;
        axis.setX(1.0f);
        axis.setY(0.0f);
        axis.setZ(0.0f);
    }
}

// has problems when dorProduct becomes 0.99999999 => 1
float Pose::getAngleBetweenDegrees(const QQuaternion &q1, const QQuaternion &q2)
{
    QQuaternion qn1 = q1.normalized();
    QQuaternion qn2 = q2.normalized();

    float dotProduct = qn1.x() * qn2.x() + qn1.y() * qn2.y() + qn1.z() * qn2.z() + qn1.scalar() * qn2.scalar();

    // http://www.gamedev.net/topic/522465-quaternion-dot-product-question/
    // when negating q1 and recomputing, we receive -dotProduct, so lets just flip the sign of dotProduct
    if(dotProduct < 0.0) dotProduct *= -1.0f;

    return RAD2DEG(2.0 * acos(dotProduct));
}

/* old version, same problems with dotProduct becoming < 0
float Pose::getAngleBetweenDegrees(const QQuaternion &q1, const QQuaternion &q2)
{
    QQuaternion diff = Pose::inverse(q1) * q2;

    float angle;
    QVector3D axis;
    rotationToAngleAxis(diff, angle, axis);

    return angle;
}*/

float Pose::getAngleBetweenDegrees(const Pose &p1, const Pose &p2)
{
    const QQuaternion q1 = p1.getOrientation();
    const QQuaternion q2 = p2.getOrientation();

    return getAngleBetweenDegrees(q1, q2);
}

QDebug operator<<(QDebug dbg, const Pose &pose)
{
    dbg.nospace() << pose.toString(false);

    return dbg.space();
}

const QString Pose::toString(bool verbose) const
{
    const QVector3D position = getPosition();

    const QString precisionString = verbose ? getFlagsString() : QString::number(precision);

    return QString("pose t%1 (%2/%3/%4) YPR (%5/%6/%7) VEL (%8/%9/%10) PR %11 CO %12")
            .arg(timestamp)
            .arg(position.x(), 0, 'f', 2)
            .arg(position.y(), 0, 'f', 2)
            .arg(position.z(), 0, 'f', 2)
            .arg(getYawDegrees(), 0, 'f', 2)
            .arg(getPitchDegrees(), 0, 'f', 2)
            .arg(getRollDegrees(), 0, 'f', 2)
            .arg(mVelocity.x(), 0, 'f', 3)
            .arg(mVelocity.y(), 0, 'f', 3)
            .arg(mVelocity.z(), 0, 'f', 3)
            .arg(precisionString)
            .arg(covariances, 0, 'f', 2);
}

const QString Pose::getFlagsString() const
{
    QString flags;
    if(precision & Pose::AttitudeAvailable) flags.append("+ATT"); else flags.append("-ATT");
    if(precision & Pose::CorrectionAgeLow) flags.append(" +COR"); else flags.append(" -COR");
    if(precision & Pose::RtkFixed) flags.append(" +RTK"); else flags.append(" -RTK");
    if(precision & Pose::ModeIntegrated) flags.append(" +INT"); else flags.append(" -INT");
    if(precision & Pose::HeadingFixed) flags.append(" +HDG"); else flags.append(" -HDG");
    return flags;
}

bool Pose::isSufficientlyPreciseForFlightControl() const
{
    return
            getAge() < 80                           // Don't use old poses!
            && precision & Pose::AttitudeAvailable  // We need at least yaw
            && precision & Pose::HeadingFixed       // Obvious
            && precision & Pose::CorrectionAgeLow   // We want precision

            // Non-integrated poses contain IMU drift. When the next integrated pose comes, there'll be a big
            // step from drifted value to ground thruth, causing trouble in the flight-controller's D-component.
            // 2013-04-23: It seems only the 5 INS poses in msec20-intervals show this problem, msec50 is fine.
            // (see e.g. "speedbased1"-logfiles). Thus, we use msec50 and do not require integrated poses.
            // 2013-04-24: This is *mostly* true, have a look at speedbased5. Use onlyintegrated poses.
            && precision & Pose::ModeIntegrated

            && precision & Pose::RtkFixed;          // When switching from RtkFixed to Differential, height can jump by >30m in 1 second(!)
}

bool Pose::isSufficientlyPreciseForSensorFusion() const
{
    if(
            precision & Pose::AttitudeAvailable &&
            precision & Pose::RtkFixed &&
            precision & Pose::CorrectionAgeLow &&
            precision & Pose::HeadingFixed &&
            //precision & Pose::ModeIntegrated &&
            covariances < Pose::maximumUsableCovariance
            )
        return true;
    else
        return false;
}

QDataStream& operator<<(QDataStream &out, const Pose &pose)
{
    out << pose.getMatrixConst();
    out << pose.timestamp;
    out << pose.precision;
    out << pose.covariances;
    out << pose.getVelocity();
    return out;
}

QDataStream& operator>>(QDataStream &in, Pose &pose)
{
    QMatrix4x4 matrix;
    QVector3D velocity;
    in >> matrix;
    in >> pose.timestamp;
    in >> pose.precision;
    in >> pose.covariances;
    in >> velocity;

    pose.setMatrix(matrix);
    pose.setVelocity(velocity);

    return in;
}
