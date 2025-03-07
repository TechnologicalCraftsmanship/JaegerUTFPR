using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
 
public class InverseKinematic
{
    //Constant values used to precaculate bigger "chunks" of the calculation
    double num, denum;
    double c1, c2, c3, c4, c5, c6, c7;
    double x;
    double y;
    double z;


    //Z displacement after the first 2 rotations
    const float zOffset = 0.1f; //in terms of arm length=2


    const int angRange = 30; //range from -i to +i

    //Used for moving forward/backward
    const int movForDeg = 45, movBacDeg = -45; 

    //Vector3 minLimits = new Vector3(-10.0f, -30.0f, 0.0f); //-(0.1838693, -0.1744623, -0.1246212)
    //Vector3 maxLimits = new Vector3(180.0f, 120.0f, 110.0f); //-(0.1838693, -0.1744623, -0.1246212)

    //Vector3 minLimits = new Vector3(-10.0f, -10.0f, 0.0f); //-(0.1838693, -0.1744623, -0.1246212)
    Vector3 minLimits = new Vector3(0.0f, 0.0f, 0.0f); //-(0.1838693, -0.1744623, -0.1246212)
    Vector3 maxLimits = new Vector3(180.0f, 120.0f, 135.0f); //-(0.1838693, -0.1744623, -0.1246212)
    
    //float cameraMinLimit = -45; //
    //float cameraMaxLimit = 45; //
    
    Vector3 currentLeftHand, currentRightHand;
    float currentRightLeg, currentLeftLeg; 
    //private float parametersScale = 5.31F; // value to convert from the real world to the equations
    Vector3 parametersScale = new Vector3(1.0F, 1.0F, 1.0F); //-(0.1838693, -0.1744623, -0.1246212)

    public bool moveLeg(bool rightLeg, Vector2 movementVector)
    {
        if(rightLeg)
        {
            if(movementVector.y > 0.1)
            {
                currentRightLeg = 100*movementVector.y;
                return true;
            } else if(movementVector.y < -0.1) {
                currentRightLeg = 100*movementVector.y;
                return true;
            } else {
                currentRightLeg = 0;
                return true;
            }
        } else {
            if(movementVector.y > 0.1)
            {
                currentLeftLeg = 100*movementVector.y;
                return true;
            } else if(movementVector.y < -0.1) {
                currentLeftLeg = 100*movementVector.y;
                return true;
            } else {
                currentLeftLeg = 0;
                return true;
            }
        }
    }
    public float obtainLeg(bool rightLeg)
    {
        if(rightLeg)
            return currentRightLeg;
        else
            return currentLeftLeg;
    }
    public void calibrate(Vector3 position)
    {
        parametersScale = new Vector3(1.0F, 1.0F, 1.0F);
        parametersScale = parametersScale * (4.0f/(position.magnitude * parametersScale.magnitude));
        //parametersScale = parametersScale * (4.0f/(position.magnitude * parametersScale.y));
        position = position * parametersScale.magnitude;        
    }
    public void updateCurrentAngles(Vector3 angles, bool rightHand)
    {
        if(rightHand)
            currentRightHand = angles;
        else
            currentLeftHand = angles;
    }
    public Vector3 obtainAngles(Vector3 position, bool rightHand)
    {
        Vector3 current, tmpPosition;
        if(position.z < 0)
            position.z = 0;

        if(rightHand)
            current = currentRightHand;
        else
        {
            current = currentLeftHand;
            //for the left hand, we must invert the obtained x axis so that the obtained angles are correct
            //note that we rotation angle of the first and second motor are also inverted physically.
            position.x = -position.x;
        }
        
        //scale the parameters so that the arm length = 4 (2+2)
        /*
        tmpPosition.x = position.x * parametersScale.x;
        tmpPosition.y = position.y * parametersScale.y;
        tmpPosition.z = position.z * parametersScale.z;*/
        tmpPosition = position * parametersScale.magnitude;        
        
        
        if(tmpPosition.magnitude > 4)
        {
            tmpPosition *= (4.0f/tmpPosition.magnitude);
        }
        position = tmpPosition;

        if(position.x == 0.0f && position.y == 0.0f && position.z == 0.0f)
        {
            Debug.Log("Return old current:" + current);
            return current;
        }
        x = position.x;
        y = position.y;
        z = position.z;
        
        /*
        //precalculate the constants used by many equations
        //and verify if everything is correct
        if(precalculate(position, rightHand) && !double.IsNaN(c5))
        {
        //    Debug.Log("Return old current:" + current);
        //    return current;
            

            //obtain all 4 possible solutions
            Vector3[] solutions = new Vector3[4];
            solutions[0] = obtain1ndSolution();
            solutions[1] = obtain2ndSolution();
            solutions[2] = obtain3ndSolution();
            solutions[3] = obtain4ndSolution();

            //Adjust the obtained angles
            for(int i = 0; i < 4; i++)
            {
                if(solutions[i].x <= -180.0f)
                    solutions[i].x += 360;
                if(solutions[i].y <= -180.0f)
                    solutions[i].y += 360;
                if(solutions[i].z <= -180.0f)
                    solutions[i].z += 360;
                    
                if(solutions[i].x < minLimits.x)
                    solutions[i].x = minLimits.x;
                if(solutions[i].x > maxLimits.x)
                    solutions[i].x = maxLimits.x;

                if(solutions[i].y < minLimits.y)
                    solutions[i].y = minLimits.y;
                if(solutions[i].y > maxLimits.y)
                    solutions[i].y = maxLimits.y;

                if(solutions[i].z < minLimits.z)
                    solutions[i].z = minLimits.z;
                if(solutions[i].z > maxLimits.z)
                    solutions[i].z = maxLimits.z;
            }


            //determine the best solution based on the distance
            int []totalDistance = new int[4];
            int minIndex = 0;
            for(int i = 0; i < 4; i++)
            {
                totalDistance[i] = (int)Math.Round(Math.Abs(current.x - solutions[i].x) + Math.Abs(current.y - solutions[i].y) + Math.Abs(current.z - solutions[i].z));
            }
            //obtain the index of the min solution
            for(int i = 1; i < 4; i++)
                if(totalDistance[minIndex] > totalDistance[i])
                    minIndex = i;

            //assign the final angles
            current = solutions[minIndex];
            
        } else {
            //*/
            ///*
            current = obtainBruteForce();
            //even brute force must have some constrains
            //if the obtained angle is outside the  physically allowable range, 
            //we must ignore it 
            //if(current.x < -80 || current.x > 200 || current.y > 185 || current.z < -20 || current.z > 110)// || 
            //(current.x < 30 &&  current.y < -5))
            //{
                //return the original values
            //    if(rightHand)
            //        return currentRightHand;
            //    else
            //        return currentLeftHand;
            //}
            //obtain the local optimal value between the calculated values and the real value (since it has an additional displacement after the second rotation)
            //current = obtainLocalOptimal(current);
            
            if(current.x < minLimits.x)
                current.x = minLimits.x;
            if(current.x > maxLimits.x)
                current.x = maxLimits.x;

            if(current.y < minLimits.y)
                current.y = minLimits.y;
            if(current.y > maxLimits.y)
                current.y = maxLimits.y;

            if(current.z < minLimits.z)
                current.z = minLimits.z;
            if(current.z > maxLimits.z)
                current.z = maxLimits.z;

            //*/
        //}

        //should never happen:
        if(float.IsNaN(current.x) || float.IsNaN(current.y) || float.IsNaN(current.z))
        {
            //if not, return the original unchanged values
            if(rightHand)
                return currentRightHand;
            else
                return currentLeftHand;
        }
        
        
            
        if(current.z > 100)
            current.z = 100;
        if(current.z < -20)
            current.z = -20;

        //update the current value of left/right hand.
        if(rightHand)
            currentRightHand = current;
        else
            currentLeftHand = current;

        return current;
    }

    //Brute force to obtain the correct angles
    private Vector3 obtainBruteForce()
    {
        Vector3 finalAngles = new Vector3(0,0,0);
        float minDistance = 9999;
        float minDistanceTA= 9999; //used to optimize the algoritm
        float curDistance;
        Vector3 tmpPost;
        float pAr, sAr, tAr;
        int count = 0;
        for(float pA = minLimits.x; pA <= maxLimits.x; pA += angRange) //angRange is used to search for the local minimum after this function
        {
            for(float sA = minLimits.y; sA <= maxLimits.y; sA += angRange)
            {                    
                minDistanceTA = 9999;
                for(float tA = minLimits.z; tA <= maxLimits.z; tA += angRange)
                {
                    pAr = (float)(pA* Math.PI/180);
                    sAr = (float)(sA* Math.PI/180);
                    tAr = (float)(tA* Math.PI/180);

                    //calculate the position for the given angles using the correct calc
                    tmpPost.x = (float)(2*Math.Cos(tAr)*Math.Sin(sAr)+2*Math.Sin(sAr));
                    tmpPost.y = (float)(-2*Math.Cos(pAr)*Math.Cos(sAr)+zOffset*Math.Sin(pAr)-2*(Math.Cos(pAr)*Math.Cos(sAr)*Math.Cos(tAr)-Math.Sin(pAr)*Math.Sin(tAr)));
                    tmpPost.z = (float)(zOffset*Math.Cos(pAr)+2*Math.Cos(sAr)*Math.Sin(pAr)-2*(-Math.Cos(sAr)*Math.Cos(tAr)*Math.Sin(pAr)-Math.Cos(pAr)*Math.Sin(tAr)));
                    //obtain the distance to the correct position
                    tmpPost.x -= (float)x;
                    tmpPost.y -= (float)y;
                    tmpPost.z -= (float)z;
                    curDistance = (float)Math.Sqrt(tmpPost.x*tmpPost.x+tmpPost.y*tmpPost.y+tmpPost.z*tmpPost.z);

                    if(minDistance > curDistance ||
                    (minDistance == curDistance && (pA*pA + sA*sA + tA*tA) <  (finalAngles.x*finalAngles.x + finalAngles.y*finalAngles.y + finalAngles.z*finalAngles.z)) )
                    {
                        minDistance = curDistance;
                        finalAngles.x = pA;
                        finalAngles.y = sA;
                        finalAngles.z = tA;
                    }
                    count++;
                    //check if the last iteration of tA were better than this current one
                    if(minDistanceTA < curDistance)
                    {
                        break; //stop the tA iteration
                    } else
                        minDistanceTA = curDistance;
                    
                }
            }
        }
        return finalAngles;
    }


    //Obtain the optimal value between the obtained 
    private Vector3 obtainLocalOptimal(Vector3 calcAng)
    {
        Vector3 finalAngles = calcAng;
        float minDistance = 9999;
        float minDistanceTA= 9999; //used to optimize the algoritm
        float curDistance;
        Vector3 tmpPost;
        float pAr, sAr, tAr;
        int count = 0;
        for(float pA = calcAng.x-angRange; pA <= calcAng.x + angRange; pA+=5)
        {
            if(pA < minLimits.x)
                continue;
            if(pA > maxLimits.x)
                break;
            for(float sA = calcAng.y-angRange; sA <= calcAng.y + angRange; sA+=5)
            {
                if(sA < minLimits.y)
                    continue;
                if(sA > maxLimits.y)
                    break;
                    
                minDistanceTA = 9999;
                for(float tA = calcAng.z-angRange; tA <= calcAng.z + angRange; tA+=5)
                {
                    if(tA < minLimits.z)
                        continue;
                    if(tA > maxLimits.z)
                        break;
                    pAr = (float)(pA* Math.PI/180);
                    sAr = (float)(sA* Math.PI/180);
                    tAr = (float)(tA* Math.PI/180);

                    //calculate the position for the given angles using the correct calc
                    tmpPost.x = (float)(2*Math.Cos(tAr)*Math.Sin(sAr)+2*Math.Sin(sAr));
                    tmpPost.y = (float)(-2*Math.Cos(pAr)*Math.Cos(sAr)+zOffset*Math.Sin(pAr)-2*(Math.Cos(pAr)*Math.Cos(sAr)*Math.Cos(tAr)-Math.Sin(pAr)*Math.Sin(tAr)));
                    tmpPost.z = (float)(zOffset*Math.Cos(pAr)+2*Math.Cos(sAr)*Math.Sin(pAr)-2*(-Math.Cos(sAr)*Math.Cos(tAr)*Math.Sin(pAr)-Math.Cos(pAr)*Math.Sin(tAr)));
                    //obtain the distance to the correct position
                    tmpPost.x -= (float)x;
                    tmpPost.y -= (float)y;
                    tmpPost.z -= (float)z;
                    curDistance = (float)Math.Sqrt(tmpPost.x*tmpPost.x+tmpPost.y*tmpPost.y+tmpPost.z*tmpPost.z);

                    if(minDistance > curDistance ||
                    (minDistance == curDistance && (pA*pA + sA*sA + tA*tA) <  (finalAngles.x*finalAngles.x + finalAngles.y*finalAngles.y + finalAngles.z*finalAngles.z)) )
                    {
                        minDistance = curDistance;
                        finalAngles.x = pA;
                        finalAngles.y = sA;
                        finalAngles.z = tA;
                    }
                    count++;
                    //check if the last iteration of tA were better than this current one
                    if(minDistanceTA < curDistance)
                    {
                        break; //stop the tA iteration
                    } else
                        minDistanceTA = curDistance;
                }
            }
        }
        return finalAngles;
    }

    /*Precalculates the constants used in other calculations*/
    private bool precalculate(Vector3 position, bool rightHand)
    {
        //having z=0 returns inf.
        if(z == 0)
            z=0.000001;

        try { 
            c1 = Math.Pow(x,4)+2*Math.Pow(x,2)*Math.Pow(y,2)+Math.Pow(y,4)+2*Math.Pow(x,2)*Math.Pow(z,2)+2*Math.Pow(y,2)*Math.Pow(z,2)+Math.Pow(z,4);
            c2 = -16*Math.Pow(x,2);
            c3 = 16*Math.Pow(y,2);
            c4 = 16*Math.Pow(z,2);
            c5 = Math.Sqrt(c2+c1)/Math.Sqrt(c1);
            c6=c3+c4;
        }
        catch (Exception e)
        {
            Debug.Log("Failed to precalculate:" + e.Message);
            return false;
        }
        return true;
    }
    private Vector3 obtain1ndSolution()
    {
        Vector3 ang;
        ang.x = 0.0f;
        ang.y = 0.0f;
        ang.z = 0.0f; 

        double sqrtTerm1;//, sqrtTerm2;

        sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        if(sqrtTerm1 < 0) 
            sqrtTerm1 = 0;
        denum = (1/(2*c6))*(8*x*x*y*c5+8*y*y*y*c5+8*y*z*z*c5-
        Math.Sqrt(sqrtTerm1));

        
        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
        num = -(1/(4*z))*(x*x*c5+y*y*c5+z*z*c5-((16*x*x*y*y/c6)*c5)-
        ((16*y*y*y*y/c6)*c5)-((c3*z*z/c6)*c5)+
        ((2*y/(c3+c4))*Math.Sqrt(sqrtTerm1)));
        ang.x = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        denum = -c5;
        num = 4*x/(x*x+y*y+z*z);
        ang.y = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        
        denum = (1/8.0f)*(-8+x*x+y*y+z*z);

        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
            
        num = -(1/(8*z))*
        (x*x*y*c5+y*y*y*c5+y*z*z*c5-(16*x*x*y*y*y/c6)*c5-(16*y*y*y*y*y/c6)*c5-(16*x*x*y*z*z/c6)*c5-
        (32*y*y*y*z*z/c6)*c5-(16*y*z*z*z*z/c6)*c5+
        (2*y*y/(c3+c4))*
        Math.Sqrt(sqrtTerm1)+
        (2*z*z/(c3+c4))*
        Math.Sqrt(sqrtTerm1)
        );
        ang.z = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        return ang;
    }
    private Vector3 obtain2ndSolution()
    {
        Vector3 ang;
        ang.x = 0.0f;
        ang.y = 0.0f;
        ang.z = 0.0f; 

        double sqrtTerm1;//, sqrtTerm2;

        sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        if(sqrtTerm1 < 0) 
            sqrtTerm1 = 0;
        denum = (1/(2*c6))*(8*x*x*y*c5+8*y*y*y*c5+8*y*z*z*c5+Math.Sqrt(sqrtTerm1));

        
        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
        num = -(1/(4*z))*(x*x*c5+y*y*c5+z*z*c5-((16*x*x*y*y/c6)*c5)-
        ((16*y*y*y*y/c6)*c5)-((c3*z*z/c6)*c5)-
        ((2*y/(c3+c4))*Math.Sqrt(sqrtTerm1)));
        ang.x = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        denum = -c5;
        num = 4*x/(x*x+y*y+z*z);
        ang.y = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        
        denum = (1/8.0f)*(-8+x*x+y*y+z*z);

        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
            
        num = -(1/(8*z))*
        (x*x*y*c5+y*y*y*c5+y*z*z*c5-(16*x*x*y*y*y/c6)*c5-(16*y*y*y*y*y/c6)*c5-(16*x*x*y*z*z/c6)*c5-
        (32*y*y*y*z*z/c6)*c5-(16*y*z*z*z*z/c6)*c5-
        (2*y*y/(c3+c4))*
        Math.Sqrt(sqrtTerm1)-
        (2*z*z/(c3+c4))*
        Math.Sqrt(sqrtTerm1)
        );
        ang.z = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        return ang;
    }
    private Vector3 obtain3ndSolution()
    { 
        Vector3 ang;
        ang.x = 0.0f;
        ang.y = 0.0f;
        ang.z = 0.0f; 

        double sqrtTerm1;//, sqrtTerm2;

        sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(8*x*x*y*c5+8*y*y*y*c5+8*y*z*z*c5,2);
        if(sqrtTerm1 < 0) 
            sqrtTerm1 = 0;
        denum = (1/(2*c6))*(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5-Math.Sqrt(sqrtTerm1));

        
        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
        num = -(1/(4*z))*(-x*x*c5-y*y*c5-z*z*c5+((16*x*x*y*y/c6)*c5)+
        ((16*y*y*y*y/c6)*c5)+((c3*z*z/c6)*c5)+
        ((2*y/(c3+c4))*Math.Sqrt(sqrtTerm1)));
        ang.x = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        denum = c5;
        num = 4*x/(x*x+y*y+z*z);
        ang.y = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        
        denum = (1/8.0f)*(-8+x*x+y*y+z*z);

        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
            
        num = -(1/(8*z))*
        (-x*x*y*c5-y*y*y*c5-y*z*z*c5+(16*x*x*y*y*y/c6)*c5+(16*y*y*y*y*y/c6)*c5+(16*x*x*y*z*z/c6)*c5+
        (32*y*y*y*z*z/c6)*c5+(16*y*z*z*z*z/c6)*c5+
        (2*y*y/(c3+c4))*
        Math.Sqrt(sqrtTerm1)+
        (2*z*z/(c3+c4))*
        Math.Sqrt(sqrtTerm1)
        );
        ang.z = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        return ang;
    }
    private Vector3 obtain4ndSolution()
    {
        Vector3 ang;
        ang.x = 0.0f;
        ang.y = 0.0f;
        ang.z = 0.0f; 

        double sqrtTerm1;//, sqrtTerm2;

        sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(8*x*x*y*c5+8*y*y*y*c5+8*y*z*z*c5,2);
        if(sqrtTerm1 < 0) 
            sqrtTerm1 = 0;
        denum = (1/(2*c6))*(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5+Math.Sqrt(sqrtTerm1));

        
        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
        num = -(1/(4*z))*(-x*x*c5-y*y*c5-z*z*c5+((16*x*x*y*y/c6)*c5)+
        ((16*y*y*y*y/c6)*c5)+((c3*z*z/c6)*c5)-
        ((2*y/(c3+c4))*Math.Sqrt(sqrtTerm1)));
        ang.x = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        denum = c5;
        num = 4*x/(x*x+y*y+z*z);
        ang.y = (float)(Math.Atan2(num,denum) * 180/Math.PI);

        
        denum = (1/8.0f)*(-8+x*x+y*y+z*z);

        //equal to the first one: sqrtTerm1 = -4*c6*(c2+c1-c4)+Math.Pow(-8*x*x*y*c5-8*y*y*y*c5-8*y*z*z*c5,2);
        //if(sqrtTerm1 < 0) 
        //    sqrtTerm1 = 0;
            
        num = -(1/(8*z))*
        (-x*x*y*c5-y*y*y*c5-y*z*z*c5+(16*x*x*y*y*y/c6)*c5+(16*y*y*y*y*y/c6)*c5+(16*x*x*y*z*z/c6)*c5+
        (32*y*y*y*z*z/c6)*c5+(16*y*z*z*z*z/c6)*c5-
        (2*y*y/(c3+c4))*
        Math.Sqrt(sqrtTerm1)-
        (2*z*z/(c3+c4))*
        Math.Sqrt(sqrtTerm1)
        );
        ang.z = (float)(Math.Atan2(num,denum) * 180/Math.PI);


        return ang;
    }
}
 