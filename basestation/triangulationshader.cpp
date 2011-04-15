
#version 120
#extension GL_EXT_geometry_shader4 : enable


#define EPSILON  0.000001

struct ITRIANGLE{
   int p1,p2,p3;
};

struct IEDGE{
   int p1,p2;
};

struct XYZ{
   float x,y,z;
};

bool CircumCircle(float xp, float yp, float x1,float y1, float x2, float y2, float x3,float y3, out float xc, out float yc, out float rsqr);

void Triangulate(int nv, XYZ pxyz[10], out ITRIANGLE v[40], out int ntri)
{
        bool complete[40];
        IEDGE edges[40];
        int nedge = 0;
        int trimax,emax = 200;
        int status = 0;

   bool inside;
   int i,j,k;
   float xp,yp,x1,y1,x2,y2,x3,y3,xc,yc,r;
   float xmin,xmax,ymin,ymax,xmid,ymid;
   float dx,dy,dmax;

   /* Allocate memory for the completeness list, flag for each triangle
   trimax = 4 * nv;
   if ((complete = malloc(trimax*sizeof(int))) == NULL) {
      status = 1;
      goto skip;
   }*/

   /* Allocate memory for the edge list
   if ((edges = malloc(emax*(long)sizeof(IEDGE))) == NULL) {
      status = 2;
      goto skip;
   }*/



   /*
      Find the maximum and minimum vertex bounds.
      This is to allow calculation of the bounding triangle
   */
   xmin = pxyz[0].x;
   ymin = pxyz[0].y;
   xmax = xmin;
   ymax = ymin;
   for (i=1;i<nv;i++) {
      if (pxyz[i].x < xmin) xmin = pxyz[i].x;
      if (pxyz[i].x > xmax) xmax = pxyz[i].x;
      if (pxyz[i].y < ymin) ymin = pxyz[i].y;
      if (pxyz[i].y > ymax) ymax = pxyz[i].y;
   }
   dx = xmax - xmin;
   dy = ymax - ymin;
   dmax = (dx > dy) ? dx : dy;
   xmid = (xmax + xmin) / 2.0;
   ymid = (ymax + ymin) / 2.0;

   /*
      Set up the supertriangle
      This is a triangle which encompasses all the sample points.
      The supertriangle coordinates are added to the end of the
      vertex list. The supertriangle is the first triangle in
      the triangle list.
   */
   pxyz[nv+0].x = xmid - 20 * dmax;
   pxyz[nv+0].y = ymid - dmax;
   pxyz[nv+0].z = 0.0;
   pxyz[nv+1].x = xmid;
   pxyz[nv+1].y = ymid + 20 * dmax;
   pxyz[nv+1].z = 0.0;
   pxyz[nv+2].x = xmid + 20 * dmax;
   pxyz[nv+2].y = ymid - dmax;
   pxyz[nv+2].z = 0.0;
   v[0].p1 = nv;
   v[0].p2 = nv+1;
   v[0].p3 = nv+2;
   complete[0] = false;
   ntri = 1;



   /*
      Include each point one at a time into the existing mesh
   */
   for (i=0;i<nv;i++) {

      xp = pxyz[i].x;
      yp = pxyz[i].y;
      nedge = 0;

      /*
         Set up the edge buffer.
         If the point (xp,yp) lies inside the circumcircle then the
         three edges of that triangle are added to the edge buffer
         and that triangle is removed.
      */
      for (j=0;j<ntri;j++) {
         if (complete[j])
            continue;
         x1 = pxyz[v[j].p1].x;
         y1 = pxyz[v[j].p1].y;
         x2 = pxyz[v[j].p2].x;
         y2 = pxyz[v[j].p2].y;
         x3 = pxyz[v[j].p3].x;
         y3 = pxyz[v[j].p3].y;
         inside = CircumCircle(xp,yp,x1,y1,x2,y2,x3,y3,xc,yc,r);
         if (xc < xp && ((xp-xc)*(xp-xc)) > r)
                                complete[j] = true;

         if (inside) {
            /* Check that we haven't exceeded the edge list size
            if (nedge+3 >= emax) {
               emax += 100;
               if ((edges = realloc(edges,emax*(long)sizeof(IEDGE))) == NULL) {
                  status = 3;
                  goto skip;
               }
            }*/

            edges[nedge+0].p1 = v[j].p1;
            edges[nedge+0].p2 = v[j].p2;
            edges[nedge+1].p1 = v[j].p2;
            edges[nedge+1].p2 = v[j].p3;
            edges[nedge+2].p1 = v[j].p3;
            edges[nedge+2].p2 = v[j].p1;
            nedge += 3;
            v[j] = v[ntri-1];
            complete[j] = complete[ntri-1];
            ntri--;
            j--;
         }
      }

      /*
         Tag multiple edges
         Note: if all triangles are specified anticlockwise then all
               interior edges are opposite pointing in direction.
      */
      for (j=0; j<nedge-1; j++) {
         for (k=j+1; k<nedge; k++) {
                 if ((edges[j].p1 == edges[k].p2) && (edges[j].p2 == edges[k].p1)) {
               edges[j].p1 = -1;
               edges[j].p2 = -1;
               edges[k].p1 = -1;
               edges[k].p2 = -1;
            }
            /* Shouldn't need the following, see note above */
            if ((edges[j].p1 == edges[k].p1) && (edges[j].p2 == edges[k].p2)) {
               edges[j].p1 = -1;
               edges[j].p2 = -1;
               edges[k].p1 = -1;
               edges[k].p2 = -1;
            }
         }
      }

      /*
         Form new triangles for the current point
         Skipping over any tagged edges.
         All edges are arranged in clockwise order.
      */
      for (j=0; j < nedge; j++) {
         if (edges[j].p1 < 0 &#0124;&#0124; edges[j].p2 < 0)
            continue;
         if (ntri >= trimax) {
            status = 4;
            return;
         }
         v[ntri].p1 = edges[j].p1;
         v[ntri].p2 = edges[j].p2;
         v[ntri].p3 = i;
         complete[ntri] = false;
         ntri++;
      }
   }

   /*
      Remove triangles with supertriangle vertices
      These are triangles which have a vertex number greater than nv
   */
   for (i=0; i < ntri; i++) {
      if (v[i].p1 >= nv &#0124;&#0124; v[i].p2 >= nv &#0124;&#0124; v[i].p3 >= nv) {
         v[i] = v[ntri - 1];
         ntri--;
         i--;
      }
   }
}


bool CircumCircle(float xp,float yp, float x1,float y1,float x2,float y2,float x3,float y3, out float xc, out float yc, out float rsqr)
{
   float m1,m2,mx1,mx2,my1,my2;
   float dx,dy,drsqr;
   float fabsy1y2 = abs(y1-y2);
   float fabsy2y3 = abs(y2-y3);

   /* Check for coincident points */
   if (fabsy1y2 < EPSILON && fabsy2y3 < EPSILON)
       return false;

   if (fabsy1y2 < EPSILON) {
      m2 = - (x3-x2) / (y3-y2);
      mx2 = (x2 + x3) / 2.0;
      my2 = (y2 + y3) / 2.0;
      xc = (x2 + x1) / 2.0;
      yc = m2 * (xc - mx2) + my2;
   } else if (fabsy2y3 < EPSILON) {
      m1 = - (x2-x1) / (y2-y1);
      mx1 = (x1 + x2) / 2.0;
      my1 = (y1 + y2) / 2.0;
      xc = (x3 + x2) / 2.0;
      yc = m1 * (xc - mx1) + my1;
   } else {
      m1 = - (x2-x1) / (y2-y1);
      m2 = - (x3-x2) / (y3-y2);
      mx1 = (x1 + x2) / 2.0;
      mx2 = (x2 + x3) / 2.0;
      my1 = (y1 + y2) / 2.0;
      my2 = (y2 + y3) / 2.0;
      xc = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
      if (fabsy1y2 > fabsy2y3) {
         yc = m1 * (xc - mx1) + my1;
      } else {
         yc = m2 * (xc - mx2) + my2;
      }
   }

   dx = x2 - xc;
   dy = y2 - yc;
   rsqr = dx*dx + dy*dy;

   dx = xp - xc;
   dy = yp - yc;
   drsqr = dx*dx + dy*dy;

   return drsqr <= rsqr;
}

void emitLineLoop(vec4 position0, vec4 position1, vec4 position2, vec4 color){
        gl_Position = position0;
        gl_FrontColor = color;
        EmitVertex();
        gl_Position = position1;
        gl_FrontColor = color;
        EmitVertex();
        EndPrimitive();

        gl_Position = position1;
        gl_FrontColor = color;
        EmitVertex();
        gl_Position = position2;
        gl_FrontColor = color;
        EmitVertex();
        EndPrimitive();

        gl_Position = position2;
        gl_FrontColor = color;
        EmitVertex();
        gl_Position = position0;
        gl_FrontColor = color;
        EmitVertex();
        EndPrimitive();
}


void main(){
        int nv = 3;
        XYZ pxyz[10];
        ITRIANGLE v[40];
        int ntri;

        pxyz[0].x = gl_PositionIn[0].x;
        pxyz[0].y = gl_PositionIn[0].y;
        pxyz[0].z = gl_PositionIn[0].z;

        pxyz[1].x = gl_PositionIn[1].x;
        pxyz[1].y = gl_PositionIn[1].y;
        pxyz[1].z = gl_PositionIn[1].z;

        pxyz[2].x = gl_PositionIn[2].x;
        pxyz[2].y = gl_PositionIn[2].y;
        pxyz[2].z = gl_PositionIn[2].z;

        pxyz[3].x = mix(pxyz[0].x, mix(pxyz[1].x, pxyz[2].x, 0.5), 0.667);
        pxyz[3].y = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].y, 0.5), 0.667);
        pxyz[3].z = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].z, 0.5), 0.667);

        pxyz[4].x = mix(pxyz[0].x, mix(pxyz[1].x, pxyz[2].x, 0.3), 0.667);
        pxyz[4].y = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].y, 0.5), 0.8);
        pxyz[4].z = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].z, 0.5), 0.4);


        pxyz[5].x = mix(pxyz[0].x, mix(pxyz[1].x, pxyz[2].x, 0.7), 0.2);
        pxyz[5].y = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].y, 0.7), 0.5);
        pxyz[5].z = mix(pxyz[0].y, mix(pxyz[1].y, pxyz[2].z, 0.2), 0.6);


        Triangulate(4, pxyz, v, ntri);

        vec4 position0, position1, position2, color;

        for(int i = 0; i < ntri; i++){
                position0 = vec4(pxyz[v[i].p1].x, pxyz[v[i].p1].y, pxyz[v[i].p1].z, 1);
                position1 = vec4(pxyz[v[i].p2].x, pxyz[v[i].p2].y, pxyz[v[i].p2].z, 1);
                position2 = vec4(pxyz[v[i].p3].x, pxyz[v[i].p3].y, pxyz[v[i].p3].z, 1);
                if(v[0].p1 == 0){
                        color = vec4(1, 0, 0, 1);
                }else{
                        color = vec4(0, 1, 0, 1);
                }

                emitLineLoop(position0, position1, position2, color);
        }
}
