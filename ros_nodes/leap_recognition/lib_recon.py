import numpy as np
from scipy.spatial.distance import cdist
import pickle


joint_n=21# 41 entrambi, 21 solo un joint per osso
joint_d=3

def preparing_frame(frame):
    frame_extracted = extract_joint_positions_from_frame(frame)  # estraggo da json a lista

    frame_jcded = JCD(frame_extracted)  # da lista a jcd


    return frame_jcded


def JCD(p):  # immagine della distanza euclidea dei joint
    # upper triangle index with offset 1, which means upper triangle without diagonal
    #C = Configuration_dataset()
    p = np.copy(np.array(p).reshape([joint_n, joint_d]))
    # prendi solo certe distanza e fai fisher map
    iu = np.triu_indices(joint_n, 1, joint_n)
    d_m = cdist(p, p, 'euclidean')
    d_m = d_m[iu]
    return d_m


def extract_joint_positions_from_frame(frame):
    joints = []

    for hand in frame.get('hands', []):
        keypoints = hand['hand_keypoints']

        # Palmo
        joints.extend(keypoints['palm_position'])

        # Braccio
        #joints.extend(keypoints['arm']['prev_joint'])
        #joints.extend(keypoints['arm']['next_joint'])

        # Dita
        fingers = keypoints['fingers']
        for finger_name in ['thumb', 'index', 'middle', 'ring', 'pinky']:
            for bone_name in ['metacarpal', 'proximal', 'intermediate', 'distal']:
                bone = fingers[finger_name][bone_name]
                #joints.extend(bone['prev_joint'])
                joints.extend(bone['next_joint'])

    return joints  # flat list of all joints for the frame



def recognition_gesture_frame(static_model,frame):
    pred = static_model.predict([frame])
    return pred

def net_loader(path,name):
    net_name = path+ name
    with open(net_name, 'rb') as fd:
        static_model = pickle.load(fd)

    return static_model


def separate_hands(hands):
    total_joints = len(hands)
    if total_joints == 0:
        return [], []  # No data received

    first_hand = hands[0]["hand_type"]
    if total_joints > 1:  # 2 hands
        if first_hand == "left":
            return hands[0], hands[1]
        else:
            return hands[1], hands[2]

    if first_hand == "left":  #1 hand
        return hands[0], []
    else:
        return [], hands[0]

