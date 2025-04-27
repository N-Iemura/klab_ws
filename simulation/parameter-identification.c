///////////////////////////////////////////////////////////////////
// //
// Intelligent Machine System //
// (Prameter Identification by RLS) //
// parameter-identification.c //
///////////////////////////////////////////////////////////////////
// gcc -o sample1 sample1.c -lglut32 -lglu32 -lopengl32

#include <GL/glut.h> // OenGL
#include <math.h> //
#include <stdio.h> //
#include <stdlib.h> //
#include <string.h> //

char string1[] = "- Parameter Identification (Force Control) - ";
char string2[200]; // for data output
char string3[200]; // for data output

double Fres=0.0; // force response          
double Fcmd=20.0; // force command
double Kv=10; // velocity feedback gain
double Ke=1000.0; // stiffness of environment
double De=50.0; // velocity coefficient of environment
double Xe=0.01; // position of environment
double Mv=1.9; // mass of manipulator
double Xa=0.0; // acceleration response
double Xv=0.0; // velocity response
double Xp=0.0; // position response
double sample=0.001; // sampling ratio
double time=0.0; // time counter
double Q[4][4]; // Q matrix in (11-8)
double deno; // denominator in (11-7)
double E; // error in (11-5)
double mu; // forgetting factor in (11-7), (11-8)
double alpha[4]; // identified parameter in (11-7)
double m[4];
double mTQ[4];
double Qm[4];
double graph1[600]; //
double graph2[600]; //
double graph3[600]; //

int
data_output()
{
    int i,j;
    static int count=0;
    if(count==600) count=0;
    Fcmd=20+5.0*sin(2*3.141593*1*time);

    // force control
    Xa=(Fcmd-Fres-Kv*Xv)/Mv;
    Xv+=Xa*sample;
    Xp+=Xv*sample;

    if(Xp>=Xe) Fres=Ke*(Xp-Xe)+De*Xv;

    time+=sample;
    count+=1;
 
   // parameter calculation
    m[1]=Xp; m[2]=Xv; m[3]=1;
    E=Fres-(alpha[1]*m[1]+alpha[2]*m[2]+alpha[3]*m[3]);
    deno=mu+Q[1][1]*m[1]*m[1]+Q[2][2]*m[2]*m[2]+Q[3][3]*m[3]*m[3]+(Q[2][1]+Q[1][2])*m[1]*m[2]+(Q[3][1]+Q[1][3])*m[1]*m[3]+(Q[3][2]+Q[2][3])*m[2]*m[3];

    for(i=1; i<=3; i++){
        for(j=1; j<=3; j++){
            alpha[i]=alpha[i]+E*Q[i][j]*m[j]/deno;
        }
    }

    for(i=1; i<=3; i++){
        mTQ[i]=0.0;
        for(j=1; j<=3; j++){
            mTQ[i]=mTQ[i]+m[j]*Q[j][i];
        }
    }

    for(i=1; i<=3; i++){
        Qm[i]=0.0;
        for(j=1; j<=3; j++){
            Qm[i]=Qm[i]+Q[i][j]*m[j];
        }
    }

    Q[1][1]=(Q[1][1]-Qm[1]*mTQ[1]/deno)/mu;
    Q[1][2]=(Q[1][2]-Qm[1]*mTQ[2]/deno)/mu;
    Q[1][3]=(Q[1][3]-Qm[1]*mTQ[3]/deno)/mu;
    Q[2][1]=(Q[2][1]-Qm[2]*mTQ[1]/deno)/mu;
    Q[2][2]=(Q[2][2]-Qm[2]*mTQ[2]/deno)/mu;
    Q[2][3]=(Q[2][3]-Qm[2]*mTQ[3]/deno)/mu;
    Q[3][1]=(Q[3][1]-Qm[3]*mTQ[1]/deno)/mu;
    Q[3][2]=(Q[3][2]-Qm[3]*mTQ[2]/deno)/mu;
    Q[3][3]=(Q[3][3]-Qm[3]*mTQ[3]/deno)/mu;

    graph1[(int)count]=Fres;
    graph2[(int)count]=alpha[1];
    graph3[(int)count]=alpha[2];

    glPushMatrix();
    glColor4f(1, 0, 0, 1.0);
    for(i=0; i<count; i++) PointSet2(2, i-300, 150+(int)graph1[i]);
    for(i=0; i<600; i++) PointSet2(1, i-300, 150+(int)Fcmd);
    glColor4f(0, 1, 0, 1.0); 
    for(i=0; i<count; i++) PointSet2(2, i-300, (int)graph2[i]/200);
    for(i=0; i<600; i++) PointSet2(1, i-300, (int)Ke/200);
    glColor4f(1, 1, 0, 1.0);
    for(i=0; i<count; i++) PointSet2(2, i-300, -150+(int)graph3[i]);
    for(i=0; i<600; i++) PointSet2(1, i-300, -150+(int)De);

    //PointSet(5, count-1, graph1[count-1],0);
    glPopMatrix();
    sprintf(string2,"Time=%f, Fres=%f, Xp=%f, count=%d", time, graph1[(int)count-1], Xp, count);
    sprintf(string3,"Ke=%f, De=%f, Xe=%f ¥n", alpha[1], alpha[2], alpha[3]/alpha[1]);

    glColor4f(1.000000, 0.843137, 0.000000, 1.0);
    print_character(string1,30,30);
    glColor4f(1, 0, 0, 1.0);
    print_character(string2,30,50);
    glColor4f(0, 1, 0, 1.0);
    print_character(string3,30,70);

    printf("Time=%5.5f, Ke=%5.5f, De=%5.5f, Xe=%5.5f¥n",time, alpha[1], alpha[2], alpha[3]/alpha[1]);
    return 0;
}

int main(int argc, char **argv)
{
    int i,j;

    graphics_init(argc, argv);

    window_set(600, 600, 10, 10, 3);
    window_init(0.0, 0.0, 0.0);
    glutDisplayFunc();
    gluOrtho2D(-300.0, 300.0, -300.0, 300.0);
    window_option();

    for(i=1; i<=3; i++){
        for(j=1; j<=3; j++){        pip install PyOpenGL PyOpenGL_accelerate
            Q[i][j]=0.0;
        }
    }

    for(i=1; i<=3; i++) Qm[i]=mTQ[i]=0.0;

    mu=0.99; Q[1][1]=Q[2][2]=1.0e6; Q[3][3]=1.0e6;
    alpha[1]=20000; alpha[2]=400.0; alpha[3]=5.0;

    for(i=0 ; i<600; i++)
        graph1[i]=graph2[i]=graph3[i]=0.0;

    glutMainLoop();

    return 0;
}