using System.Collections.Generic;
using System.IO;
using UnityEngine;
using System;
using SimpleJSON;
using System.Linq;

public enum BlendMode
{
    Additive,
    Replace
}

struct Directory
{
    public int parentIndex;
    public List<int> childrenIndices;
}

struct Pose
{
    public string name;
    public BlendMode blendMode;
    public List<int> corrects;
    public Dictionary<int, Matrix4x4> deltaMatrices;
}
struct BonePose
{
    public string poseName;
    public Matrix4x4 matrix;
    public BlendMode blendMode;
    public List<string> corrects;
}

[Serializable]
public struct UserPose
{
    public string poseName;
    [Range(0, 1)]
    public float poseWeight;
}

public class Skeleposer : MonoBehaviour
{
    public Animator animator;
    public string filePath;
    public SkinnedMeshRenderer meshWithBlendShapes;

    public List<UserPose> userPoses;

    Dictionary<GameObject, Matrix4x4> initialMatrices;
    Dictionary<GameObject, List<BonePose>> bonePoses; // per bone
    Dictionary<string, int> blendShapes; // name: index

    static void getPosesOrder(Dictionary<int, Directory> directories, int directoryIdx, ref List<int> poseIndices)
    {
        Directory dir;
        if (directories.TryGetValue(directoryIdx, out dir))
        {
            foreach (int childIndex in dir.childrenIndices)
            {
                if (childIndex >= 0)
                    poseIndices.Add(childIndex);
                else
                    getPosesOrder(directories, -childIndex, ref poseIndices);
            }
        }
    }

    void loadFromJson(string filePath)
    {
        Debug.Log("Loading json");

        bonePoses = new();

        string jsonStr = System.IO.File.ReadAllText(filePath);
        JSONNode json = JSON.Parse(jsonStr);

        // get bone names
        Dictionary<int, GameObject> bones = new(); // per index
        JSONObject jointsObject = json["joints"].AsObject;
        foreach (string jointIdxStr in jointsObject.Keys)
        {
            int jointIdx;
            int.TryParse(jointIdxStr, out jointIdx);
            string boneName = jointsObject[jointIdxStr];

            GameObject bone = GameObject.Find(boneName);
            if (bone != null)
            {
                bones.Add(jointIdx, bone);
                bonePoses.Add(bone, new List<BonePose>());
            }
        }

        // get directories
        Dictionary<int, Directory> directories = new();

        JSONObject directoriesObject = json["directories"].AsObject;
        foreach (string dirIdxStr in directoriesObject.Keys)
        {
            int dirIdx;
            int.TryParse(dirIdxStr, out dirIdx);

            JSONObject dirObject = directoriesObject[dirIdxStr].AsObject;

            Directory dir = new();
            dir.parentIndex = dirObject["directoryParentIndex"].AsInt;

            dir.childrenIndices = new();
            foreach (int childIndex in dirObject["directoryChildrenIndices"].AsArray.Values)
                dir.childrenIndices.Add(childIndex);

            directories.Add(dirIdx, dir);
        }
        
        // get poses
        Dictionary<int, Pose> poses = new();

        JSONObject posesObject = json["poses"].AsObject;
        foreach (string poseIdxStr in posesObject.Keys)
        {
            int poseIdx;
            int.TryParse(poseIdxStr, out poseIdx);

            JSONObject poseObject = posesObject[poseIdxStr].AsObject;

            Pose pose = new();
            pose.name = poseObject["poseName"];
            pose.blendMode = (BlendMode)poseObject["poseBlendMode"].AsInt;

            // get corrects
            pose.corrects = new();
            if (poseObject.HasKey("corrects"))
            {
                JSONArray correctsArray = poseObject["corrects"].AsArray;
                for (int i = 0; i < correctsArray.Count; i++)
                    pose.corrects.Add(correctsArray[i]);
            }
            
            // get delta matrices
            JSONObject deltaMatricesObject = poseObject["poseDeltaMatrices"].AsObject;
            
            pose.deltaMatrices = new();
            foreach (string deltaMatrixIdx in deltaMatricesObject.Keys)
            {
                int deltaIdx;
                int.TryParse(deltaMatrixIdx, out deltaIdx);

                Matrix4x4 deltaMatrix = new();
                
                JSONArray deltaMatrixArray = deltaMatricesObject[deltaMatrixIdx].AsArray;
                for (int i = 0; i < deltaMatrixArray.Count; i++)
                    deltaMatrix[i] = deltaMatrixArray[i].AsFloat;

                deltaMatrix[12] /= 100.0f; // default unit in Unity is meter, cm is in Maya
                deltaMatrix[13] /= 100.0f;
                deltaMatrix[14] /= 100.0f;

                pose.deltaMatrices.Add(deltaIdx, deltaMatrix);
            }            
            
            poses.Add(poseIdx, pose);
        }

        // get all poses indices in a correct order
        List<int> poseIndices = new();
        getPosesOrder(directories, 0, ref poseIndices);

        // get pose per bone list, like bone1:[pose1, pose2, pose3], bone2:[pose1, pose4]
        foreach (int poseIdx in poseIndices)
        {
            Pose pose = poses[poseIdx];

            foreach (int deltaIdx in pose.deltaMatrices.Keys)
            {
                GameObject bone;
                if (bones.TryGetValue(deltaIdx, out bone))
                {
                    BonePose bpose = new();
                    bpose.poseName = pose.name;
                    bpose.matrix = pose.deltaMatrices[deltaIdx];
                    bpose.blendMode = pose.blendMode;

                    bpose.corrects = new();
                    foreach (int correctPoseIdx in pose.corrects) // corrects are pose names
                        bpose.corrects.Add(poses[correctPoseIdx].name);

                    bonePoses[bone].Add(bpose);
                }
                else
                    Debug.Log("Cannot find bone at index "+deltaIdx);
            }
        }
    }


    void getBlendShapes()
    {
        blendShapes = new();

        if (meshWithBlendShapes == null)
            return;

        Mesh mesh = meshWithBlendShapes.sharedMesh;
        for (int i = 0; i < mesh.blendShapeCount; i++)
        {
            string targetName = mesh.GetBlendShapeName(i);
            string localTargetName = targetName.Split(".").Last(); // strip blendShape node name, like blendShape1.targetName
            blendShapes.Add(localTargetName, i);
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        initialMatrices = new();

        string fullFilePath = Path.Combine(Application.streamingAssetsPath, filePath);
        if (File.Exists(fullFilePath)) 
        {
            loadFromJson(fullFilePath);

            getBlendShapes();

            // save matrices
            foreach (var (bone, _) in bonePoses)
            {
                Matrix4x4 m = Matrix4x4.TRS(bone.transform.localPosition, bone.transform.localRotation, bone.transform.localScale);
                initialMatrices.Add(bone, m);
            }
        }
        else
            Debug.LogError("Cannot find "+ fullFilePath);

        if (animator != null)
            animator.enabled = false;  // disable attached animator
    }

    private void Update()
    {
        // restore initial matrices
        foreach (var (bone, m) in initialMatrices)
        {
            bone.transform.SetLocalPositionAndRotation(m.GetPosition(), m.rotation);
            bone.transform.localScale = m.lossyScale;
        }

        if (animator != null)
            animator.Update(Time.deltaTime);

        updateSkeleposer();
    }

    void updateSkeleposer()
    {
        if (userPoses.Count == 0)
            return;

        Dictionary<string, float> posesWeights = new(); // user poses' weights and weights that are calculated during the computation
        foreach (var userPose in userPoses)
            posesWeights.Add(userPose.poseName, userPose.poseWeight);

        foreach (var (bone, bonePoses) in bonePoses)
        {
            Transform boneTransform = bone.transform;

            Vector3 translation = boneTransform.localPosition;
            Quaternion rotation = boneTransform.localRotation;
            Vector3 scale = boneTransform.localScale;

            foreach (var bonePose in bonePoses)
            {
                float poseWeight = posesWeights.GetValueOrDefault(bonePose.poseName, 0.0f);
                
                if (bonePose.corrects.Count>0) // if corrects found
                {
                    poseWeight = float.MaxValue; // find lowest weight
                    foreach (string correctPoseName in bonePose.corrects)
					{
                        float w = posesWeights.GetValueOrDefault(correctPoseName, 0.0f);
                        if (w < poseWeight)
                            poseWeight = w;
                    }
                }

                posesWeights[bonePose.poseName] = poseWeight; // save user pose weight

                if (poseWeight > 0.001)
                {
                    if (bonePose.blendMode == BlendMode.Additive)
                    {
                        translation += bonePose.matrix.GetPosition() * poseWeight;
                        rotation = Quaternion.Slerp(rotation, rotation * bonePose.matrix.rotation, poseWeight);
                        scale = Vector3.Lerp(scale, Vector3.Scale(scale, bonePose.matrix.lossyScale), poseWeight);
                    }

                    else if (bonePose.blendMode == BlendMode.Replace)
                    {
                        translation = Vector3.Lerp(translation, bonePose.matrix.GetPosition(), poseWeight);
                        rotation = Quaternion.Slerp(rotation, bonePose.matrix.rotation, poseWeight);
                        scale = Vector3.Lerp(scale, bonePose.matrix.lossyScale, poseWeight);
                    }
                }
            }

            boneTransform.localPosition = translation;
            boneTransform.localRotation = rotation;
            boneTransform.localScale = scale;
        }

        // activate blend shapes
        if (blendShapes.Count > 0)
        {
            foreach (var (poseName, poseWeight) in posesWeights)
            {
                int blendShapeIdx;
                if (blendShapes.TryGetValue(poseName, out blendShapeIdx))
                    meshWithBlendShapes.SetBlendShapeWeight(blendShapeIdx, poseWeight * 100);
            }
        }
    }
}
