/*
*   satcollision.h
*
*   2D SAT collisions detection C library made by Marc Montagut - @MarcMDE  
*
*
*   Copyright (c) 2016 Marc Montagut
*
*   This software is provided "as-is", without any express or implied warranty. In no event
*   will the authors be held liable for any damages arising from the use of this software.
*
*   Permission is granted to anyone to use this software for any purpose, including commercial
*   applications, and to alter it and redistribute it freely, subject to the following restrictions:
*
*     1. The origin of this software must not be misrepresented; you must not claim that you
*     wrote the original software. If you use this software in a product, an acknowledgment
*     in the product documentation would be appreciated but is not required.
*
*     2. Altered source versions must be plainly marked as such, and must not be misrepresented
*     as being the original software.
*
*     3. This notice may not be removed or altered from any source distribution.
*
*/

#ifndef SATCOLLISION_H
#define SATCOLLISION_H

#include "raylib.h"

#ifdef __cplusplus
extern "C" {            // Prevents name mangling of functions
#endif


// Structs
// ---------------------------
typedef struct SATBox
{
    Vector2 position; // Should be in the middle of points. 
    Vector2 points[4];
    Vector2 size;
    float rotation;
}SATBox;

typedef struct SATTri // Is supposed to be a ("x") symetric triangle
{
    Vector2 position; // Should be in the middle of points. 
    Vector2 points[3];
    Vector2 size;
    float rotation;
}SATTri;

typedef struct SATRegPoly
{
    Vector2 position; // Should be in the middle of points. 
    Vector2 *points; // free points before finishing the program!!! ->  free(SATRegPoly.points);
    float radius;
    int sides;
    float rotation;
}SATRegPoly;

typedef struct SATIrrPoly
{
    Vector2 position; // Should be in the middle of points. 
    Vector2 *points;
    float rotation;
}SATIrrPoly;

typedef struct Value2
{
    float a;
    float b;
}Value2;
// ----------------------------

// Functions
// ----------------------------
Texture2D CreateWhitePixelTexture ();
Vector2 GetNormal(Vector2 a, Vector2 b, bool left);
void SetNormals (Vector2 *points, Vector2 *normals, int lenght, bool left);
void RotatePoints (Vector2 *points, int lenght, Vector2 pivot, float angle);
Value2 GetProjectedMinMax (Vector2 *points, int lenght, Vector2 normal);
bool MinMaxCollide (Value2 a, Value2 b);
void InitSATBox (SATBox *box, Vector2 position, Vector2 size, float rotation);
void InitSATTri (SATTri *tri, Vector2 position, Vector2 size, float rotation);
void InitSATRegPoly (SATRegPoly *poly, Vector2 position, float radius, int sides, float rotation);
void UpdateSATBox (SATBox *box, Vector2 position, Vector2 size, float rotation);
void UpdateSATTri (SATTri *tri, Vector2 position, Vector2 size, float rotation);
void UpdateAASATBoxPosition (SATBox *box, Vector2 position);
void UpdateAASATTriPosition (SATTri *tri, Vector2 position);
bool SATPolysCollide (Vector2 *p1Points, int p1Lenght, Vector2 *p2Points, int p2Lenght);
bool SATPolysNCollide (Vector2 *p1Points, Vector2 *p1Normals, int p1Lenght, Vector2 *p2Points, Vector2 *p2Normals, int p2Lenght);
bool SATPolyPolyNCollide (Vector2 *p1Points, int p1Lenght, Vector2 *p2Points, Vector2 *p2Normals, int p2Lenght);
// ----------------------------


#ifdef __cplusplus
}
#endif

#endif // CSATCOLLISION_H