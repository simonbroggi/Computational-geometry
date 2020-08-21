using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Habrador_Computational_Geometry;

public class DelaunayIncrementalController : MonoBehaviour
{
    public List<Transform> points;
    Mesh triangulatedMesh;
    MeshFilter meshFilter;
    MeshRenderer meshRenderer;
    public Rect bounds;
    AABB2 normalizingBox; // Rect in Habrador? what's the difference to Rect??
    float dMax;
    HalfEdgeData2 triangleData_normalized;

    void Start()
    {
        InitializeMeshComponents();

        normalizingBox = new AABB2(bounds.xMin, bounds.xMax, bounds.yMin, bounds.yMax);
        dMax = HelpMethods.CalculateDMax(normalizingBox);

        // Triangle2 superTriangle = new Triangle2(new MyVector2(-10f, -10f), new MyVector2(10f, -10f), new MyVector2(0f, 10f));
        Triangle2 quadTri1 = new Triangle2(new MyVector2(0f, 0f), new MyVector2(1f, 0f), new MyVector2(0f, 1f));
        Triangle2 quadTri2 = new Triangle2(new MyVector2(1f, 0f), new MyVector2(1f, 1f), new MyVector2(0f, 1f));
        

        //Create the triangulation data with a quad
        HashSet<Triangle2> triangles = new HashSet<Triangle2>();
        triangles.Add(quadTri1);
        triangles.Add(quadTri2);

        //Change to half-edge data structure
        triangleData_normalized = new HalfEdgeData2();
        _TransformBetweenDataStructures.Triangle2ToHalfEdge2(triangles, triangleData_normalized);
    }

    void InitializeMeshComponents()
    {
        meshFilter = GetComponent<MeshFilter>();
        if(meshFilter == null)
        {
            meshFilter = gameObject.AddComponent<MeshFilter>();
        }
        if(meshFilter.mesh == null)
        {
            meshFilter.mesh = new Mesh();
        }
        triangulatedMesh = meshFilter.mesh;

        meshRenderer = GetComponent<MeshRenderer>();
        if(meshRenderer == null)
        {
            meshRenderer = gameObject.AddComponent<MeshRenderer>();
        }
    }

    [ContextMenu("Add random point")]
    void AddRandomPoint()
    {
        //These are for display purposes only
        int missedPoints = 0;
        int flippedEdges = 0;
        InsertNewPointInTriangulation(new MyVector2(Random.value, Random.value), triangleData_normalized, ref missedPoints, ref flippedEdges);
    }

    private void Update()
    {
        if(Input.GetKey(KeyCode.Space))
        {
            AddRandomPoint();
        }

        //UnNormalize
        HalfEdgeData2 triangleData = HelpMethods.UnNormalize(triangleData_normalized, normalizingBox, dMax);

        //From half-edge to triangle
        HashSet<Triangle2> triangles_2d = _TransformBetweenDataStructures.HalfEdge2ToTriangle2(triangleData);

        //From 2d to 3d
        HashSet<Triangle3> triangles_3d = new HashSet<Triangle3>();

        foreach (Triangle2 t in triangles_2d)
        {
            triangles_3d.Add(new Triangle3(t.p1.ToMyVector3(), t.p2.ToMyVector3(), t.p3.ToMyVector3()));
        }

        triangulatedMesh = _TransformBetweenDataStructures.Triangle3ToCompressedMesh(triangles_3d, triangulatedMesh); 
    }


    //Insert a new point in the triangulation we already have, so we need at least one triangle
    public static void InsertNewPointInTriangulation(MyVector2 p, HalfEdgeData2 triangulationData, ref int missedPoints, ref int flippedEdges)
    {
        //Step 5. Insert the new point in the triangulation
        //Find the existing triangle the point is in
        HalfEdgeFace2 f = PointTriangulationIntersection.TriangulationWalk(p, null, triangulationData);

        //We couldnt find a triangle maybe because the point is not in the triangulation?
        if (f == null)
        {
            missedPoints += 1;
        }

        //Delete this triangle and form 3 new triangles by connecting p to each of the vertices in the old triangle
        HalfEdgeHelpMethods.SplitTriangleFaceAtPoint(f, p, triangulationData);


        //Step 6. Initialize stack. Place all triangles which are adjacent to the edges opposite p on a LIFO stack
        //The report says we should place triangles, but it's easier to place edges with our data structure
        Stack<HalfEdge2> trianglesToInvestigate = new Stack<HalfEdge2>();

        AddTrianglesOppositePToStack(p, trianglesToInvestigate, triangulationData);


        //Step 7. Restore delaunay triangulation
        //While the stack is not empty
        int safety = 0;

        while (trianglesToInvestigate.Count > 0)
        {
            safety += 1;

            if (safety > 1000000)
            {
                Debug.Log("Stuck in infinite loop when restoring delaunay in incremental sloan algorithm");

                break;
            }

            //Step 7.1. Remove a triangle from the stack
            HalfEdge2 edgeToTest = trianglesToInvestigate.Pop();

            //Step 7.2. Do we need to flip this edge? 
            //If p is outside or on the circumcircle for this triangle, we have a delaunay triangle and can return to next loop
            MyVector2 a = edgeToTest.v.position;
            MyVector2 b = edgeToTest.prevEdge.v.position;
            MyVector2 c = edgeToTest.nextEdge.v.position;
            
            //abc are here counter-clockwise
            if (DelaunayMethods.ShouldFlipEdgeStable(a, b, c, p))
            {
                HalfEdgeHelpMethods.FlipTriangleEdge(edgeToTest);

                //Step 7.3. Place any triangles which are now opposite p on the stack
                AddTrianglesOppositePToStack(p, trianglesToInvestigate, triangulationData);

                flippedEdges += 1;
            }
        }
    }



    //Find all triangles opposite of vertex p
    //But we will find all edges opposite to p, and from these edges we can find the triangles
    private static void AddTrianglesOppositePToStack(MyVector2 p, Stack<HalfEdge2> trianglesOppositeP, HalfEdgeData2 triangulationData)
    {
        //Find a vertex at position p and then rotate around it, triangle-by-triangle, to find all opposite edges
        HalfEdgeVertex2 rotateAroundThis = null;

        foreach (HalfEdgeVertex2 v in triangulationData.vertices)
        {
            if (v.position.Equals(p))
            {
                rotateAroundThis = v;
            }
        }

        //Which triangle is this vertex a part of, so we know when we have rotated all the way around
        HalfEdgeFace2 tStart = rotateAroundThis.edge.face;

        HalfEdgeFace2 tCurrent = null;

        int safety = 0;

        while (tCurrent != tStart)
        {
            safety += 1;

            if (safety > 10000)
            {
                Debug.Log("Stuck in endless loop when finding opposite edges in Delaunay Sloan");

                break;
            }

            //The edge opposite to p
            HalfEdge2 edgeOppositeRotateVertex = rotateAroundThis.edge.nextEdge.oppositeEdge;

            //Try to add the edge to the list iof triangles we are interested in 
            //Null might happen if we are at the border
            //A stack might include duplicates so we have to check for that as well
            if (edgeOppositeRotateVertex != null && !trianglesOppositeP.Contains(edgeOppositeRotateVertex))
            {
                trianglesOppositeP.Push(edgeOppositeRotateVertex);
            }

            //Rotate left - this assumes we can always rotate left so no holes are allowed
            //and neither can we investigate one of the vertices thats a part of the supertriangle
            //which we dont need to worry about because p is never a part of the supertriangle
            rotateAroundThis = rotateAroundThis.edge.oppositeEdge.v;

            //In which triangle are we now?
            tCurrent = rotateAroundThis.edge.face;
        }
    }

}
