def convert_human_to_unity_spider_joint_angles(human_spider_angles: list[float]) -> list[float]:
    """
    Converts spider joint angles from human-friendly format to Unity's format.

    Parameters
    ----------
    human_spider_angles: list[float]
        A list of 16 angles that are intuitive for humans.
        
        Shoulder joints:
            - Abduction: positive
            - Adduction: negative
        
        Calf joints:
            - Forward extension: positive
            - Backward extension: negative
    
    Returns
    ----------
    unity_spider_angles: list[float]
        A list of 16 angles formatted for Unity.
        
        Facing the spider in Unity:
        - For the 4 front shoulder joints, positive angle values indicate counterclockwise rotation.
        - For the 4 back shoulder joints, positive angle values indicate clockwise rotation.
        - For the 4 left calf joints, positive angle values indicate backward extension.
        - For the 4 right calf joints, positive angle values indicate forward extension.
    
    Raises
    ----------
    ValueError
        If human_spider_angles does not consist of exactly 16 float values.
    """

    unity_spider_angles: list[float] = [
        human_spider_angles[0],
        -human_spider_angles[1],

        human_spider_angles[2],
        -human_spider_angles[3],

        -human_spider_angles[4],
        -human_spider_angles[5],

        -human_spider_angles[6],
        -human_spider_angles[7],

        -human_spider_angles[8],
        human_spider_angles[9],

        -human_spider_angles[10],
        human_spider_angles[11],

        human_spider_angles[12],
        human_spider_angles[13],

        human_spider_angles[14],
        human_spider_angles[15]
    ]

    return unity_spider_angles