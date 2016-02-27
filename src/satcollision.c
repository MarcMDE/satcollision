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

void RotatePoints (Vector2 *points, Vector2 pivot, float angle)
{
    int lenght = sizeof(points)/sizeof(Vector2);
    
    for (int i=0; i<lenght; i++) Vector2Rotate(&points[i], pivot, angle);
}

Value2 GetProjectedMinMax (Vector2 *points, Vector2 normal)
{
    int lenght;
    float current, min, max;
    
    lenght = sizeof(points)/sizeof(Vector2);
    current = Vector2DotProduct(points[0], normal);
    min = current;
    max = min;
    
    for (int i=1; i<lenght; i++)
    {
        current = Vector2DotProduct(points[i], normal);
        
        if (current < min) current = min;
        if (current > max) current = max;
    }
    
    return (Value2){min, max};
}

bool MinMaxCollide (Value2 a, Value2 b)
{
    if (a.b < b.a || b.b < a.a) return false; // If max < min, then, there is no collision
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
    
    if (box->rotation != 0) RotatePoints(box->points, box->position, box->rotation);
}

void InitSATTri (SATTri *tri, Vector2 position, Vector2 size, float rotation)
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
    
    if (tri->rotation != 0) RotatePoints(tri->points, tri->position, tri->rotation);
}

void InitSATRegPoly (SATRegPoly *poly, Vector2 position, float radius, int sides, float rotation)
{
    poly->position = position;
    poly->radius = radius;
    poly->sides = sides;
    poly->rotation = rotation;
    
    for (int i=0; i<sides; i++)
    {
        poly->points[i] = Vector2FloatProduct(Vector2Up(), poly->radius);
        Vector2Rotate(&poly->points[0], poly->position, (360/sides)*i);
    }
    
    if (poly->rotation != 0) RotatePoints(poly->points, poly->position, poly->rotation);
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
        
        if (box->rotation != 0)RotatePoints(box->points, box->position, box->rotation);
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
        
        if (tri->rotation != 0) RotatePoints(tri->points, tri->position, tri->rotation);
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

bool PolysCollide (Vector2 *p1Points, Vector2 *p2Points)
{
    int p1Lenght;
    Vector2 normal;
    
    p1Lenght = sizeof(p1Points)/sizeof(Vector2);
    
    for (int i=0; i<p1Lenght; i++)
    {
        normal = GetNormal(p1Points[i], p1Points[i+1], true);
        if (!MinMaxCollide(GetProjectedMinMax(p1Points, normal), GetProjectedMinMax(p2Points, normal))) return false;
    }
    // Check first-last point normal
    normal = GetNormal(p1Points[0], p1Points[p1Lenght-1], true);
    if (!MinMaxCollide(GetProjectedMinMax(p1Points, normal), GetProjectedMinMax(p2Points, normal))) return false;
    
    int p2Lenght;
    p2Lenght = sizeof(p2Points)/sizeof(Vector2);
    
    for (int i=0; i<p2Lenght; i++)
    {
        normal = GetNormal(p2Points[i], p2Points[i+1], true);
        if (!MinMaxCollide(GetProjectedMinMax(p1Points, normal), GetProjectedMinMax(p2Points, normal))) return false;
    }
    // Check first-last point normal
    normal = GetNormal(p2Points[0], p2Points[p2Lenght-1], true);
    if (!MinMaxCollide(GetProjectedMinMax(p1Points, normal), GetProjectedMinMax(p2Points, normal))) return false;
    
    return true;
}


