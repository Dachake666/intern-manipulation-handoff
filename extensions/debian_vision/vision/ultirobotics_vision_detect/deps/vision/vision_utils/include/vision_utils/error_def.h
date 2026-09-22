#pragma once
#ifndef VISION_UTILS_ERROR_DEF_H
#define VISION_UTILS_ERROR_DEF_H
typedef enum {
    // 通用状态
    ERR_OK = 0,                  // 成功

    // 相机基础错误 (-1 ~ -20)
    ERR_CAM_NOT_INIT = -1,       // 相机未初始化
    ERR_CAM_INIT_FAILED = -2,    // 相机初始化失败
    ERR_CAM_DISCONNECTED = -3,   // 相机断线 / 长时间获取图像失败
    ERR_CAM_SEARCH_FAILED = -4,  // 搜索相机失败
    ERR_CAM_NOT_FOUND = -5,      // 未检测到相机
    ERR_CAM_SPEC_NOT_FOUND = -6, // 未搜索到指定的相机
    ERR_CAM_OPEN_FAILED = -7,    // 相机打开失败
    ERR_CAM_SET_PARAM_FAILED = -8,// 相机初始化时设置参数失败
    ERR_CAM_SET_PARAM_ERR = -9,  // 相机参数设置异常（备用）
    ERR_CAM_DEVICE_EXCEPTION = -20, // 相机设备异常

    // 相机数据请求错误 (-30 ~ -40)
    ERR_CAM_NO_CORRESPOND = -30, // 请求相机数据时，没有对应的相机
    ERR_CAM_IMG_EMPTY = -31,     // 相机获取的图像为空 或 内参不对
    ERR_CAM_CROP_REGION_INVALID = -32, // 自动曝光区域数据不对
    ERR_CAM_AUTO_EXPOSURE_FAILED = -33,// 自动曝光失败
    ERR_CAM_AUTO_EXPOSURE_IMAGE_INVALID = -34,// 自动曝光图像格式不对
    ERR_CAM_AUTO_EXPOSURE_REGION_EMPTY = -35,// 自动曝光区域无有效像素
    ERR_CAM_AUTO_EXPOSURE_PROP_UNSUPPORTED = -36,// 相机不支持自动曝光所需属性
    ERR_CAM_AUTO_EXPOSURE_PROP_SET_FAILED = -37,// 自动曝光属性设置失败
    ERR_CAM_AUTO_EXPOSURE_PROP_GET_FAILED = -38,// 自动曝光属性读取失败
    ERR_CAM_AUTO_EXPOSURE_RANGE_GET_FAILED = -39,// 自动曝光范围读取失败
    ERR_CAM_EXPOSURE_SET_TYPE_FAILED = -40,// 曝光类型设置失败

    // 相机服务器错误 (-40 ~ -50)
    ERR_CAM_SERVER_UNREACH = -41,// 联系不到相机服务器
    ERR_CAM_DATA_TIMEOUT = -42,  // 求取相机数据超时
    ERR_CAM_SERVER_DATA_ERR = -43,// 从相机服务器获取到的相机数据不对
    ERR_CAM_IN_USE = -44,        // 当前相机在使用中，稍后再求取数据
    ERR_CAM_SERVER_DATA_SIZE_ERR = -45,// 相机服务器返回图像数据长度不对

    // 视觉检测服务器错误 (-50 ~ -60)
    ERR_VISION_SERVER_NO_FOUND = -51, // 请求视觉检测时，无对应检测服务器

    // 视觉检测通用错误 (-90 ~ -99)
    ERR_LOCAL_IMG_DATA_INVALID = -90, // 本地图像数据不对
    ERR_LOCAL_IMG_NOT_FOUND = -91,    // 本地图像未找到
    ERR_APP_RUNTIME_FAILED = -97,     // 程序运行失败
    ERR_APP_INVALID_ARG = -98,        // 程序启动参数不对
    ERR_VISION_CAM_DATA_ERR = -99,    // 视觉检测时发现相机数据不对

    // 物料框 / 目标检测错误 (-100 ~ -120)
    ERR_BOX_NOT_DETECTED = -100,      // 未检测到物料框
    ERR_BOX_INVALID = -101,           // 未检测到正确的物料框
    ERR_TARGET_NOT_FOUND = -102,      // 未找到正确的目标
    ERR_TARGET_POINT_CLOUD_LACK = -103,// 检测到的目标点云过少
    ERR_TARGET_POINT_CLOUD_NOISE = -104,// 目标点云噪声过多
    ERR_TARGET_EMPTY_BOX = -106,      // 未检测到目标，判断为空箱
    ERR_BOX_VERTEX_FAILED = -110,     // 物料框四个顶点检测失败
    ERR_TARGET_SEGMENT_FAILED = -112, // 目标点云分割失败
    ERR_TARGET_RESULT_INVALID = -113, // 目标检测结果数据不对
    ERR_DETECT_AREA_OCCLUDE = -115,   // 检测区域上方有物体遮挡
    ERR_BOX_OCCLUDE = -116,           // 物料框上方有物体遮挡

    // 手眼标定错误 (-130 ~ -150)
    ERR_HAND_EYE_CAM_DATA_INVALID = -130, // 手眼标定相机数据不对
    ERR_HAND_EYE_REQUEST_INVALID = -131,  // 手眼标定请求类型不对
    ERR_HAND_EYE_MARKER_TOO_FEW = -133,   // 识别到的标定板数量不足
    ERR_HAND_EYE_POSE_TOO_FEW = -134,     // 手眼标定位姿数量不足
    ERR_HAND_EYE_CONFIG_LOAD_FAILED = -135,// 手眼标定配置加载失败
    ERR_HAND_EYE_CONFIG_SAVE_FAILED = -136,// 手眼标定配置保存失败
    ERR_HAND_EYE_RESULT_INVALID = -137,   // 手眼标定结果误差过大
    ERR_HAND_EYE_TRANSFORM_FAILED = -138, // 手眼标定变换矩阵计算失败

    // 通讯 ZMQ 错误 (-200 ~ -210)
    ERR_ZMQ_NOT_INIT = -200,          // zmq 未初始化
    ERR_ULTINET_DETECT_ZMQ_ERR = -201,// ultinet 检测结果为 0 或 zmq 通讯失败
    ERR_ZMQ_COMM_FAILED = -202,       // zmq 通讯失败
    ERR_ULTINET_DATA_INVALID = -203,   // ultinet 检测的数据不对
    ERR_ZMQ_RECONNECTING= -204,           //zmq 连接失败重新连接中
    ERR_ZMQ_SEND_ING = -205            //zmq 通讯中

} ErrorCode;

#endif