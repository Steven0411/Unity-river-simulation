using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using UnityEngine.InputSystem;
using UnityEngine.ParticleSystemJobs;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;

[BurstCompile]
public class waterAlgorithm : MonoBehaviour
{
    // viscosity of water at 20°C
    private float viscosity = 0.000001002f;

    private NativeArray<float> Vx;
    private NativeArray<float> Vy;
    private NativeArray<float> Vz;

    private NativeArray<float> PreVx;
    private NativeArray<float> PreVy;
    private NativeArray<float> PreVz;

    private NativeArray<bool> Ground;
    private NativeArray<bool> Selected;
    private NativeArray<bool> ControlPoints;

    private Texture2D waterTexture;
    private Gradient speedGradient;

    public int N = 50;
    public int yN = 5;
    public float iter = 100;
    public float dt = 0.01f;
    public float flowSpeed = 100;

    private float interpolate = 0f;
    private int timesPressed = 0;

    private float3 start = new float3();
    private float3 end = new float3();
    private float3 cont1 = new float3();
    private float3 cont2 = new float3();

    public GameObject Groundobject;
    public GameObject Waterobject;
    public GameObject Debrisobject;
    private List<GameObject> debris;

    private ParticleSystem watereffect;
    private ParticleSystem.Particle[] particles;

    int Index(int x, int y, int z)
    {
        // there is an error somewhere fix it
        return x + y * N + z * N * yN;
    }
    int Index(int x, int z)
    {
        // there is an error somewhere fix it
        return x + z * N;
    }

    public void fluidSolverLoop()
    {
        diffuse(1, PreVx, Vx, viscosity, dt);
        diffuse(2, PreVy, Vy, viscosity, dt);
        diffuse(3, PreVz, Vz, viscosity, dt);

        project(PreVx, PreVy, PreVz, Vx, Vy);

        advect(1, Vx, PreVx, PreVx, PreVy, PreVz, dt);
        advect(2, Vy, PreVy, PreVx, PreVy, PreVz, dt);
        advect(3, Vz, PreVz, PreVx, PreVy, PreVz, dt);

        project(Vx, Vy, Vz, PreVx, PreVy);
    }

    void advect(int boundaryFlag, NativeArray<float> density, NativeArray<float> densityPrev,
    NativeArray<float> velocityX, NativeArray<float> velocityY, NativeArray<float> velocityZ, float dt)
    {
        float timeStepScaleX = dt * (N - 2);
        float timeStepScaleY = dt * (yN - 2);
        float timeStepScaleZ = dt * (N - 2);

        float backtraceWeight0, backtraceWeight1, interpWeightY0, interpWeightY1, interpWeightZ0, interpWeightZ1;
        float backtraceX, backtraceY, backtraceZ;

        float gridSizeFloat = N;
        float ySizeFloat = yN;
        int x, y, z;

        for(z = 1; z < N - 1; z++){
            for(x = 1; x < N - 1; x++){
                if (Ground[Index(x, z)])
                {
                    density[Index(x, 1, z)] = 0;
                    continue;
                }
                for(y = 1; y < yN - 1; y++){

                    backtraceX = x - timeStepScaleX * velocityX[Index(x, y, z)];
                    backtraceY = y - timeStepScaleY * velocityY[Index(x, y, z)];
                    backtraceZ = z - timeStepScaleZ * velocityZ[Index(x, y, z)];

                    backtraceX = Mathf.Clamp(backtraceX, 0.5f, gridSizeFloat - 2.0f);
                    backtraceY = Mathf.Clamp(backtraceY, 0.5f, ySizeFloat - 2.0f);
                    backtraceZ = Mathf.Clamp(backtraceZ, 0.5f, gridSizeFloat - 2.0f);

                    int gridIndexX0 = Mathf.FloorToInt(backtraceX);
                    int gridIndexX1 = gridIndexX0 + 1;
                    int gridIndexY0 = Mathf.FloorToInt(backtraceY);
                    int gridIndexY1 = gridIndexY0 + 1;
                    int gridIndexZ0 = Mathf.FloorToInt(backtraceZ);
                    int gridIndexZ1 = gridIndexZ0 + 1;

                    backtraceWeight0 = 1.0f - (backtraceX - gridIndexX0) * 0.1f;
                    backtraceWeight1 = backtraceX - gridIndexX0;
                    interpWeightY0 = 1.0f - (backtraceY - gridIndexY0);
                    interpWeightY1 = backtraceY - gridIndexY0;
                    interpWeightZ0 = 1.0f - (backtraceZ - gridIndexZ0);
                    interpWeightZ1 = backtraceZ - gridIndexZ0;

                    density[Index(x, y, z)] =
                        backtraceWeight0 * (interpWeightY0 * (interpWeightZ0 * densityPrev[Index(gridIndexX0, gridIndexY0, gridIndexZ0)]
                                                            + interpWeightZ1 * densityPrev[Index(gridIndexX0, gridIndexY0, gridIndexZ1)])
                                          + interpWeightY1 * (interpWeightZ0 * densityPrev[Index(gridIndexX0, gridIndexY1, gridIndexZ0)]
                                                            + interpWeightZ1 * densityPrev[Index(gridIndexX0, gridIndexY1, gridIndexZ1)]))
                      + backtraceWeight1 * (interpWeightY0 * (interpWeightZ0 * densityPrev[Index(gridIndexX1, gridIndexY0, gridIndexZ0)]
                                                            + interpWeightZ1 * densityPrev[Index(gridIndexX1, gridIndexY0, gridIndexZ1)])
                                          + interpWeightY1 * (interpWeightZ0 * densityPrev[Index(gridIndexX1, gridIndexY1, gridIndexZ0)]
                                                            + interpWeightZ1 * densityPrev[Index(gridIndexX1, gridIndexY1, gridIndexZ1)]));
                }
            }
        }
        set_bnd(boundaryFlag, density);
    }

    void diffuse(int b, NativeArray<float> x, NativeArray<float> x0, float diff, float dt)
    {
        float a = dt * diff * (N - 1) * (yN - 1) * (N - 1);
        gaussSeidelSolver(b, x, x0, a, 1 + 6 * a);
    }

    void gaussSeidelSolver(int b, NativeArray<float> x, NativeArray<float> x0, float a, float c)
    {
        float cRecip = 1.0f / c;
        for (int m = 0; m < iter; m++){
            for (int k = 1; k < N - 1; k++){
                for(int i = 1; i < N - 1; i++){
                    if (Ground[Index(i, k)])
                    {
                        x[Index(i, 1, k)] = 0;
                        continue;
                    }
                    for(int j = 1; j < yN - 1; j++){
                        
                        x[Index(i, j, k)] = (x0[Index(i, j, k)] + a * (x[Index(i + 1, j, k)] + x[Index(i - 1, j, k)]
                                                       + x[Index(i, j + 1, k)] + x[Index(i, j - 1, k)]
                                                       + x[Index(i, j, k + 1)] + x[Index(i, j, k - 1)])) * cRecip;
                    }
                }
            }
            set_bnd(b, x);
        }
    }

    void project(NativeArray<float> velocX, NativeArray<float> velocY, NativeArray<float> velocZ, NativeArray<float> p, NativeArray<float> div)
    {
        for (int z = 1; z < N - 1; z++){
            for (int x = 1; x < N - 1; x++){
                if(Ground[Index(x, z)])
                {
                    continue;
                }
                for (int y = 1; y < yN - 1; y++){
                    div[Index(x, y, z)] = -0.5f * (velocX[Index(x + 1, y, z)] - velocX[Index(x - 1, y, z)]
                                          + velocY[Index(x, y + 1, z)] - velocY[Index(x, y - 1, z)]
                                          + velocZ[Index(x, y, z + 1)] - velocZ[Index(x, y, z - 1)]) / N;
                    p[Index(x, y, z)] = 0;
                }
            }
        }
        set_bnd(0, div);
        set_bnd(0, p);
        gaussSeidelSolver(0, p, div, 1, 6);

        for (int z = 1; z < N - 1; z++){
            for (int x = 1; x < N - 1; x++){
                if(Ground[Index(x, z)])
                {
                    continue;
                }
                for (int y = 1; y < yN - 1; y++){
                    velocX[Index(x, y, z)] -= 0.5f * (p[Index(x + 1, y, z)] - p[Index(x - 1, y, z)]) * N;
                    velocY[Index(x, y, z)] -= 0.5f * (p[Index(x, y + 1, z)] - p[Index(x, y - 1, z)]) * yN;
                    velocZ[Index(x, y, z)] -= 0.5f * (p[Index(x, y, z + 1)] - p[Index(x, y, z - 1)]) * N;
                }
            }
        }
        set_bnd(1, velocX);
        set_bnd(2, velocY);
        set_bnd(3, velocZ);
    }

    void addVelocity(float xflow, float yflow, float zflow)
    {
        for (int z = 1; z < N - 1; z++)
        {
            if(Ground[Index(1, z)])
            {
                continue;
            }
            for (int y = 1; y < yN - 1; y++)
            {
                Vx[Index(1, y, z)] += xflow;
                Vy[Index(1, y, z)] += yflow;
                Vz[Index(1, y, z)] += zflow;
            }
        }
    }

    void set_bnd(int axis, NativeArray<float> coord)
    {
        for (int z = 0; z < N - 2; z++){
            for (int x = 0; x < N - 2; x++){
                if (Ground[Index(x, z)])
                {
                    coord[Index(x, 1, z)] = 0;
                    continue;
                }
                for (int y = 0; y < N - 2; y++){

                    if (x == N - 2 && axis == 1)
                    {
                        coord[Index(x, y, z)] = flowSpeed;
                    }
                    if (x == N - 2 && (axis == 2 || axis == 3))
                    {
                        coord[Index(x, y, z)] = 0;
                    }
                }
            }
        } 
    }

    void setGround()
    {
        for (int z = 0; z < N; z++){
            for (int x = 0; x < N; x++){
                Ground[Index(x, z)] = true;
                if (Selected[Index(x, z)])
                {
                    Ground[Index(x, z)] = false;
                }
            }
        }
    }

    void Start()
    {
        Vx = new NativeArray<float>(N * yN * N, Allocator.Persistent);
        Vy = new NativeArray<float>(N * yN * N, Allocator.Persistent);
        Vz = new NativeArray<float>(N * yN * N, Allocator.Persistent);

        PreVx = new NativeArray<float>(N * yN * N, Allocator.Persistent);
        PreVy = new NativeArray<float>(N * yN * N, Allocator.Persistent);
        PreVz = new NativeArray<float>(N * yN * N, Allocator.Persistent);

        Ground = new NativeArray<bool>(N * N, Allocator.Persistent);
        Selected = new NativeArray<bool>(N * N, Allocator.Persistent);
        ControlPoints = new NativeArray<bool>(N * N, Allocator.Persistent);
              
        setGround(); 
        createSpeedGradient();

        waterTexture = new Texture2D(N, N, TextureFormat.RGBA32, false);
        waterTexture.filterMode = FilterMode.Point;
        waterTexture.wrapMode = TextureWrapMode.Clamp;
        waterTexture.Apply();

        watereffect = GetComponent<ParticleSystem>();
        watereffect.Stop();

        debris = new List<GameObject>();
    }

    void Update()
    {
        addVelocity(flowSpeed, 0, 0);
        fluidSolverLoop();
        applyWaterTexture();
        customizeRiver();

        if (timesPressed < 5)
        {
            selectRiver();
        }
        else if (Input.GetMouseButtonDown(0))
        {
            SpawnDebris();
        }
        DestroyDebris();

        particles = new ParticleSystem.Particle[watereffect.particleCount];
        int particleCount = watereffect.GetParticles(particles);
        Vector3 flowpush = new Vector3(5, -9.81f, 0);

        if (particleCount == 0)
        {
            return;
        }

        NativeArray<Vector3> velocities = new NativeArray<Vector3>(particleCount, Allocator.TempJob);

        for (int i = 0; i < particleCount; i++)
        {
            velocities[i] = particles[i].velocity;
        }

        ParticleJob job = new ParticleJob();
        {
            job.velocities = velocities;
            job.Vx = Vx;
            job.Vy = Vy;
            job.Vz = Vz;
            job.flowpush = flowpush;
            job.N = N;
            job.yN = yN;
        }
        JobHandle jobHandle = job.Schedule(watereffect, particleCount);
        jobHandle.Complete();



        for (int i = 0; i < particleCount; i++)
        {
            particles[i].velocity = velocities[i];
        }

        watereffect.SetParticles(particles, particleCount);
        velocities.Dispose();
    }

    void createSpeedGradient()
    {
        speedGradient = new Gradient();
        
        GradientColorKey[] colorKeys = new GradientColorKey[5];
        colorKeys[0].color = Color.red;
        colorKeys[0].time = 0.0f;
        colorKeys[1].color = Color.yellow;
        colorKeys[1].time = 0.25f;
        colorKeys[2].color = Color.green;
        colorKeys[2].time = 0.5f;
        colorKeys[3].color = Color.cyan;
        colorKeys[3].time = 0.75f;
        colorKeys[4].color = Color.blue;
        colorKeys[4].time = 1.0f;

        GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
        alphaKeys[0].alpha = 1.0f;
        alphaKeys[0].time = 0.0f;
        alphaKeys[1].alpha = 1.0f;
        alphaKeys[1].time = 0.0f;

        speedGradient.SetKeys(colorKeys, alphaKeys);
    }

    void applyWaterTexture()
    {
        Color[] pixels = new Color[N * N];

        for (int x = 0; x < N; x++)
        {
            for (int z = 0; z < N; z++)
            {
                int index = x + z * N;
                if(x == 0 || x == N - 1 || z == 0 || z == N - 1)
                {
                    pixels[index] = Color.black;
                    continue;
                }
                if(Selected[Index(x, z)] == true || ControlPoints[Index(x, z)])
                {
                    pixels[index] = Color.yellow;
                    continue;
                }   
                if(Ground[Index(x, z)] == true)
                {
                    pixels[index] = Color.grey;
                    continue;
                }

                Vector3 waterVector = new Vector3(Vx[Index(x, 1, z)], Vz[Index(x, 1, z)]);
                float waterSpeed = waterVector.magnitude;
                Color waterColor = speedGradient.Evaluate(waterSpeed);
                pixels[index] = waterColor;
            }
        }
        waterTexture.SetPixels(pixels);
        waterTexture.Apply();
        GetComponent<Renderer>().material.mainTexture = waterTexture;  
    }

    void selectRiver()
    {
        Mouse mouse = Mouse.current;
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        if (Physics.Raycast(ray, out RaycastHit hit))
        {
            Renderer rend = hit.collider.GetComponent<Renderer>();

            Vector2 pixelcoord = hit.textureCoord;
            int Xcoord = Mathf.FloorToInt(pixelcoord.x * waterTexture.width);
            int Zcoord = Mathf.FloorToInt(pixelcoord.y * waterTexture.height);

            if (Xcoord > 0 && Xcoord < waterTexture.width - 1 && Zcoord > 0 && Zcoord < waterTexture.height - 1)
            {
                waterTexture.SetPixel(Xcoord, Zcoord, Color.yellow);
                waterTexture.Apply();

                if (mouse.leftButton.wasPressedThisFrame)
                {
                    ControlPoints[Index(Xcoord, Zcoord)] = true;

                    if (timesPressed == 4)
                    {   
                        timesPressed++;

                        // creates an offest for the particle spawner to prevent it spawning in the Ground
                        int i = 0;
                        for (int k = 1; k < N-1; k++)
                        {
                            if (Selected[Index(1, k)])
                            {
                                if (i == 0)
                                {
                                    start.z = k;
                                }
                                i++;    
                            }
                        }
                        start.z += i/2;

                        setGround();
                        resetSelected();
                        spawnGround();

                        // sets spawn position of ParticleSystem
                        float z = ((start.z - 1) * (10f))/(N - 3) - 5f;
                        Vector3 startpos = new Vector3(4.8f, 0.2f, -z);
                        var shape = watereffect.shape;
                        shape.position = startpos;

                        // sets spawn rotation of the ParticleSystem
                        Vector3 startrot = shape.rotation;
                        Vector3 dir = BezierCurve(0.1f) - new Vector3(start.x, start.y, start.z);
                        Quaternion lookrot = Quaternion.LookRotation(-dir);
                        shape.rotation = lookrot.eulerAngles;

                        watereffect.Play();

                        start = new float3();
                        end = new float3();
                        cont1 = new float3();
                        cont2 = new float3();
                        return;
                    }
                    if (timesPressed == 3)
                    {
                        cont2 = new float3(Xcoord, 1, Zcoord);
                        timesPressed++;
                    }
                    if (timesPressed == 2)
                    {
                        cont1 = new float3(Xcoord, 1, Zcoord);
                        timesPressed++;
                    }
                    if (timesPressed == 1)
                    {
                        end = new float3(Xcoord, 1, Zcoord);
                        timesPressed++;
                    }
                    if (timesPressed == 0)
                    {
                        start = new float3(Xcoord, 1, Zcoord);                       
                        timesPressed++;
                    }
                }
            }
        }
    }

    void resetSelected()
    {
        for (int x = 0; x < N; x++)
        {
            for (int z = 0; z < N; z++)
            {
                Selected[Index(x, z)] = false;
                ControlPoints[Index(x, z)] = false;
            }
        }
    }

    Vector3 scaleToWorld(float3 coord)
    {
        float worldmax = 119.1f;
        float worldmin = -119.1f;
        float gridmax = N - 2;
        float gridmin = 1;
        
        float newx = Mathf.Lerp(worldmin, worldmax, (coord.x - gridmin) / (gridmax - gridmin));
        float newz = Mathf.Lerp(worldmin, worldmax, (coord.z - gridmin) / (gridmax - gridmin));

        Vector3 newcoord = new Vector3(newx, coord.y, newz);

        return newcoord; 
    }

    Vector3 scaleToGrid(float3 coord)
    {
        float worldmax = 119.1f;
        float worldmin = -119.1f;
        float gridmax = N -2;
        float gridmin = 1;
        
        float newx = Mathf.Lerp(gridmin, gridmax, (coord.x - worldmin) / (worldmax - worldmin));
        float newz = Mathf.Lerp(gridmin, gridmax, (coord.z - worldmin) / (worldmax - worldmin));

        Vector3 newcoord = new Vector3(newx, coord.y, newz);

        return newcoord;
    }

    void makeRiverShape(float3 coord)
    {
        int x = Mathf.FloorToInt(coord.x);
        int z = Mathf.FloorToInt(coord.z);

        Selected[Index(x, z)] = true;

        if (x != N - 1)
        {
            Selected[Index(x + 1, z)] = true;
            Selected[Index(x + 1, z + 1)] = true;
            Selected[Index(x + 1, z - 1)] = true;
        }
        if (x != 1)
        {
            Selected[Index(x - 1, z)] = true;
            Selected[Index(x - 1, z + 1)] = true;
            Selected[Index(x - 1, z - 1)] = true;
        }
        if (z != N - 1)
        {
            Selected[Index(x, z + 1)] = true;
            Selected[Index(x + 1, z + 1)] = true;
            Selected[Index(x - 1, z + 1)] = true;
        }
        if (z != 1)
        {
            Selected[Index(x, z - 1)] = true;
            Selected[Index(x + 1, z - 1)] = true;
            Selected[Index(x - 1, z - 1)] = true;
        }
    }

    void customizeRiver()
    {
        if (start.Equals(float3.zero) || end.Equals(float3.zero) || cont1.Equals(float3.zero) || cont2.Equals(float3.zero))
        {
            return;
        }
        interpolate = (interpolate + Time.deltaTime * 0.2f) % 1f;
        Vector3 path = BezierCurve(interpolate);

        if(path.x >= 1 || path.z >= 1)
        {
            makeRiverShape(path);
        }
    }

    Vector3 BezierCurve(float interpolate)
    {
        Vector3 vstart = scaleToWorld(start);
        Vector3 vend = scaleToWorld(end);
        Vector3 vcont1 = scaleToWorld(cont1);
        Vector3 vcont2 = scaleToWorld(cont2);

        Vector3 ab = Vector3.Lerp(vstart, vcont1, interpolate);
        Vector3 bc = Vector3.Lerp(vcont1, vcont2, interpolate);
        Vector3 cd = Vector3.Lerp(vcont2, vend, interpolate);
        Vector3 ab_bc = Vector3.Lerp(ab, bc, interpolate);
        Vector3 bc_cd = Vector3.Lerp(bc, cd, interpolate);
        Vector3 path = scaleToGrid(Vector3.Lerp(ab_bc, bc_cd, interpolate));

        return path;
    }

    void spawnGround()
    {
        for(int x = 1; x < N-1; x++)
        {
            for (int z = 1; z < N-1; z++)
            {
                if(Ground[Index(x, z)])
                {
                    Vector3 groundpos = new Vector3(x,10,z);
                    GameObject groundobj = Instantiate(Groundobject);
                    groundobj.transform.position = scaleToWorld(groundpos);
                }
            }
        }
    }

    void SpawnDebris()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, Mathf.Infinity))
        {
            Vector3 spawnPosition = hit.point;
            spawnPosition.y = 15;
            debris.Add((GameObject)Instantiate(Debrisobject, spawnPosition, Quaternion.identity));
        }
    }

    void DestroyDebris()
    {
        for (int i = 0; i < debris.Count; i++)
        {
            Vector3 debrisPos = debris[i].transform.position;

            if (debrisPos.y <= 0)
            {
                Destroy(debris[i]);
                debris.RemoveAt(i);
            }
        }
    }
}

[BurstCompile]
struct ParticleJob : IJobParticleSystemParallelFor
{
    public NativeArray<Vector3> velocities;

    [ReadOnly] public NativeArray<float> Vx;
    [ReadOnly] public NativeArray<float> Vy;
    [ReadOnly] public NativeArray<float> Vz;
    [ReadOnly] public Vector3 flowpush;
    [ReadOnly] public int N;
    [ReadOnly] public int yN;

    public void Execute(ParticleSystemJobData particles, int i)
    {
        if (i >= particles.count || i >= velocities.Length)
        {
            return;
        }

        var particlepos = particles.positions[i];
        Vector3 gridcoord = scaleToGrid(particlepos);

        var x = math.clamp((int)math.floor(gridcoord.x), 1, N - 1);
        var y = math.clamp((int)math.floor(gridcoord.y), 1, yN - 1);
        var z = math.clamp((int)math.floor(gridcoord.z), 1, N - 1);

        int index = x + y * N + z * N * yN;

        Vector3 gridVelocity = new Vector3(Vx[index], Vy[index], Vz[index]);
        velocities[i] += (gridVelocity * 0.005f) + flowpush;
    }

    Vector3 scaleToGrid(float3 coord)
    {
        float worldmax = 119.1f;
        float worldmin = -119.1f;
        float gridmax = N - 2;
        float gridmin = 1;

        float newx = Mathf.Lerp(gridmin, gridmax, (coord.x - worldmin) / (worldmax - worldmin));
        float newz = Mathf.Lerp(gridmin, gridmax, (coord.z - worldmin) / (worldmax - worldmin));

        Vector3 newcoord = new Vector3(newx, coord.y, newz);

        return newcoord;
    }
}
