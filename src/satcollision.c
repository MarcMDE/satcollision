/*
*   satcollision.c
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

#include "satcollision.h"
#include "c2dmath.h"
#include <math.h>       // Standard math library.  
#include <stdio.h>
#include <stdlib.h>

Texture2D CreateWhitePixelTexture ()
{
   // Create a 1 white pixel texture
    Color *pixels = malloc(sizeof(Color));
    pixels[0] = WHITE;
    Image pixelImage = LoadImageEx(pixels, 1, 1);
    Texture2D texture = LoadTextureFromImage(pixelImage);
    UnloadImage(pixelImage);
    free(pixels); 
    
    return texture;
}

Vector2 GetNormal(Vector2 a, Vector2 b, bool left)
{
    Vector2 normal;
    
    normal = Vector2Sub(a, b);
    FloatSwap(&normal.x, &normal.y);
    if (left) normal.x *= -1;
    else normal.y *= -1;
    
    return normal;
}

void SetNormals (Vector2 *points, Vector2 *normals, int lenght, bool left)
{   
    for (int i=0; i<lenght-1; i++) normals[i] = GetNormal(points[i], points[i+1], left);
    normals[lenght-1] = GetNormal(points[0], points[lenght-1], left);
}

void RotatePoints (Vector2 *points, int lenght, Vector2 pivot, float angle)
{   
    for (int i=0; i<lenght; i++) Vector2Rotate(&points[i], pivot, angle);
}

Value2 GetProjectedMinMax (Vector2 *points, int lenght, Vector2 normal)
{
    float current, min, max;
    
    current = Vector2DotProduct(points[0], normal);
    min = current;
    max = current;
    
    for (int i=1; i<lenght; i++)
    {
        current = Vector2DotProduct(points[i], normal);
        
        if (current < min) min = current;
        if (current > max) max = current;
    }
    
    return (Value2){min, max};
}

bool MinMaxCollide (Value2 a, Value2 b)
{
    if (a.a > b.b || b.a > a.b) return false; // If max < min, then, there is no collision
    return true;
}

void InitSATBox (SATBox *box, Vector2 position, Vector2 size, float rotation)
{
    box->position = position;
    box->size = size;
    box->rotation = rotation;
    
    // Set half-size for faster use on points asignation
    Vector2Scale(&size, 0.5f);
    
    // Set box corners position
    box->points[0] = Vector2Sub(box->position, Vector2Product(Vector2One(), size));
    box->points[1] = Vector2Add(box->position, Vector2Product((Vector2){1, -1}, size));
    box->points[2] = Vector2Add(box->position, Vector2Product(Vector2One(), size));
    box->points[3] = Vector2Sub(box->position, Vector2Product((Vector2){1, -1}, size));
    
    if (box->rotation != 0) RotatePoints(box->points, 4, box->position, box->rotation);
}

void InitSATTri (SATTri *tri, Vector2 position, Vector2 size, float rotation)
{
    tri->position = position;
    tri->size = size;
    tri->rotation = rotation;
    
    // Set half-size for faster use on points asignation
    Vector2Scale(&size, 0.5f);
    
    // Set tri corners position
    tri->points[0] = Vector2Add(tri->position, Vector2Product((Vector2){0, -1}, size));
    tri->points[1] = Vector2Add(tri->position, Vector2Product((Vector2){1, 1}, size));
    tri->points[2] = Vector2Add(tri->position, Vector2Product((Vector2){-1, 1}, size));
    
    if (tri->rotation != 0) RotatePoints(tri->points, 3, tri->position, tri->rotation);
}

void InitSATRegPoly (SATRegPoly *poly, Vector2 position, float radius, int sides, float rotation)
{
    poly->position = position;
    poly->radius = radius;
    poly->sides = sides;
    poly->rotation = rotation;
    poly->points = malloc(sizeof(Vector2)*sides);
    
    float angle = 360.0f/(float)poly->sides;
    
    for (int i=0; i<poly->sides; i++)
    {
        poly->points[i] = Vector2FloatProduct((Vector2){0, -1}, poly->radius);
        Vector2Rotate(&poly->points[i], Vector2Zero(), angle*(float)i);
        
        poly->points[i] = Vector2Add(poly->points[i], poly->position);
    }
    
    if (poly->rotation != 0) RotatePoints(poly->points, poly->sides, poly->position, poly->rotation);
}

void UpdateSATBox (SATBox *box, Vector2 position, Vector2 size, float rotation)
{   
    if (box->position.x != position.x || box->position.y != position.y || box->size.x != size.x || box->size.y != size.y || box->rotation != rotation)
    {
        box->position = position;
        box->size = size;
        box->rotation = rotation;
        
        // Set half-size for faster use on points asignation
        Vector2Scale(&size, 0.5f);
        
        // Set box corners position
        box->points[0] = Vector2Sub(box->position, Vector2Product(Vector2One(), size));
        box->points[1] = Vector2Add(box->position, Vector2Product((Vector2){1, -1}, size));
        box->points[2] = Vector2Add(box->position, Vector2Product(Vector2One(), size));
        box->points[3] = Vector2Sub(box->position, Vector2Product((Vector2){1, -1}, size));
        
        if (box->rotation != 0) RotatePoints(box->points, 4, box->position, box->rotation);
    }
}

void UpdateSATTri (SATTri *tri, Vector2 position, Vector2 size, float rotation)
{
    if (tri->position.x != position.x || tri->position.y != position.y || tri->size.x != size.x || tri->size.y != size.y || tri->rotation != rotation)
    {
        tri->position = position;
        tri->size = size;
        tri->rotation = rotation;
        
        // Set half-size for faster use on points asignation
        Vector2Scale(&size, 0.5f);
        
        // Set tri corners position
        tri->points[0] = Vector2Add(tri->position, Vector2Product((Vector2){-1, 1}, size));
        tri->points[1] = Vector2Add(tri->position, Vector2Product((Vector2){0, -1}, size));
        tri->points[2] = Vector2Add(tri->position, Vector2Product((Vector2){1, 1}, size));
        
        if (tri->rotation != 0) RotatePoints(tri->points, 3, tri->position, tri->rotation);
    }
}

void UpdateAASATBoxPosition (SATBox *box, Vector2 position)
{
    if (box->position.x != position.x || box->position.y != position.y)
    {
        Vector2 size;
        
        box->position = position;
        size = box->size;
        
        Vector2Scale(&size, 0.5f);
        
        box->points[0] = Vector2Sub(box->position, Vector2Product(Vector2One(), size));
        box->points[1] = Vector2Add(box->position, Vector2Product((Vector2){1, -1}, size));
        box->points[2] = Vector2Add(box->position, Vector2Product(Vector2One(), size));
        box->points[3] = Vector2Sub(box->position, Vector2Product((Vector2){1, -1}, size));
    }
}

void UpdateAASATTriPosition (SATTri *tri, Vector2 position)
{
    if (tri->position.x != position.x || tri->position.y != position.y)
    {
        Vector2 size;
        
        tri->position = position;
        size = tri->size;
        
        // Set half-size for faster use on points asignation
        Vector2Scale(&size, 0.5f);
        
        // Set tri corners position
        tri->points[0] = Vector2Add(tri->position, Vector2Product((Vector2){-1, 1}, size));
        tri->points[1] = Vector2Add(tri->position, Vector2Product((Vector2){0, -1}, size));
        tri->points[2] = Vector2Add(tri->position, Vector2Product((Vector2){1, 1}, size));
    }
}

void UpdateSATRegPoly (SATRegPoly *poly, Vector2 position, float radius, float rotation)
{
    if (poly->position.x != position.x || poly->position.y != position.y || poly->radius != radius || poly->rotation != rotation)
    {
        poly->position = position;
        poly->radius = radius;
        poly->rotation = rotation;
        
        float angle = 360.0f/(float)poly->sides;
        
        for (int i=0; i<poly->sides; i++)
        {
            poly->points[i] = Vector2FloatProduct((Vector2){0, -1}, poly->radius);
            Vector2Rotate(&poly->points[i], Vector2Zero(), angle*(float)i);
            
            poly->points[i] = Vector2Add(poly->points[i], poly->position);
        }
        
        if (poly->rotation != 0) RotatePoints(poly->points, poly->sides, poly->position, poly->rotation);
    }
}

bool SATPolysCollide (Vector2 *p1Points, int p1Lenght, Vector2 *p2Points, int p2Lenght)
{
    Vector2 normal;
    
    for (int i=0; i<p1Lenght; i++)
    {
        normal = GetNormal(p1Points[i], p1Points[i+1], true);

        if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, normal), GetProjectedMinMax(p2Points, p2Lenght, normal))) return false;
    }
    // Check first-last point normal
    normal = GetNormal(p1Points[0], p1Points[p1Lenght-1], true);
    if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, normal), GetProjectedMinMax(p2Points, p2Lenght, normal))) return false;
    
    for (int i=0; i<p2Lenght; i++)
    {
        normal = GetNormal(p2Points[i], p2Points[i+1], true);
        if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, normal), GetProjectedMinMax(p2Points, p2Lenght, normal))) return false;
    }
    // Check first-last point normal
    normal = GetNormal(p2Points[0], p2Points[p2Lenght-1], true);
    if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, normal), GetProjectedMinMax(p2Points, p2Lenght, normal))) return false;
    
    return true;
}

bool SATPolysNCollide (Vector2 *p1Points, Vector2 *p1Normals, int p1Lenght, Vector2 *p2Points, Vector2 *p2Normals, int p2Lenght)
{
    for (int i=0; i<p1Lenght; i++) if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, p1Normals[i]), GetProjectedMinMax(p2Points, p2Lenght, p1Normals[i]))) return false;
    for (int i=0; i<p2Lenght; i++) if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, p2Normals[i]), GetProjectedMinMax(p2Points, p2Lenght, p2Normals[i]))) return false;
    
    return true;
}

bool SATPolyPolyNCollide (Vector2 *p1Points, int p1Lenght, Vector2 *p2Points, Vector2 *p2Normals, int p2Lenght)
{
    Vector2 p1Normal;
    
    for (int i=0; i<p1Lenght; i++)
    {
        p1Normal = GetNormal(p1Points[i], p1Points[i+1], true);

        if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, p1Normal), GetProjectedMinMax(p2Points, p2Lenght, p1Normal))) return false;
    }
    // Check first-last point normal
    p1Normal = GetNormal(p1Points[0], p1Points[p1Lenght-1], true);
    if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, p1Normal), GetProjectedMinMax(p2Points, p2Lenght, p1Normal))) return false;
    
    for (int i=0; i<p2Lenght; i++) if (!MinMaxCollide(GetProjectedMinMax(p1Points, p1Lenght, p2Normals[i]), GetProjectedMinMax(p2Points, p2Lenght, p2Normals[i]))) return false;
    
    return true;
}

bool SATPolyCircCollide (Vector2 *pPoints, Vector2 pPosition, int pLenght, Vector2 cPosition, float cRadius)
{
    Vector2 polyToCircle, polyToCircleNormalized;
    float current, max, polyToCircleMagnitude;
    
    polyToCircle = Vector2Sub(cPosition, pPosition);
    polyToCircleNormalized = polyToCircle;
    Vector2Normalize(&polyToCircleNormalized);
    polyToCircleMagnitude = Vector2Lenght(polyToCircle);
    
    max = Vector2DotProduct(Vector2Sub(pPosition, pPosition), polyToCircleNormalized);
    
    for (int i=0; i<pLenght; i++)
    {
        current = Vector2DotProduct(Vector2Sub(pPoints[i], pPosition), polyToCircleNormalized);
        if (current > max) max = current;
    }   
    
    if (polyToCircleMagnitude - max - cRadius > 0 && polyToCircleMagnitude > 0) return false;
    return true;
}