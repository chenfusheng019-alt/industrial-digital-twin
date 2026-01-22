import tensorflow as tf
import numpy as np


def create_and_save_tflite_model():
    # 创建模型
    model = tf.keras.Sequential([
        tf.keras.layers.Dense(24, input_shape=(4,), activation='relu', name='dense_1'),
        tf.keras.layers.Dense(24, activation='relu', name='dense_2'),
        tf.keras.layers.Dense(6, activation='linear', name='dense_3')
    ])

    # 编译模型
    model.compile(optimizer='adam', loss='mse')

    # 创建一个示例输入来构建模型
    dummy_input = np.zeros((1, 4), dtype=np.float32)
    _ = model(dummy_input)

    # 获取模型权重并打印信息
    weights = model.get_weights()
    print("\nModel weights information:")
    for i, w in enumerate(weights):
        print(f"Weight {i}: shape {w.shape}, total parameters: {np.prod(w.shape)}")
    print(f"Total parameters: {sum(np.prod(w.shape) for w in weights)}")

    # 转换为TFLite模型
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    # 设置转换选项
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.target_spec.supported_types = [tf.float32]

    # 转换模型
    tflite_model = converter.convert()

    # 保存模型
    with open('target_network.tflite', 'wb') as f:
        f.write(tflite_model)

    # 验证模型
    interpreter = tf.lite.Interpreter(model_content=tflite_model)
    interpreter.allocate_tensors()

    # 打印模型信息
    print("\nTFLite model information:")
    for t in interpreter.get_tensor_details():
        print(f"Tensor: {t['name']}, Shape: {t['shape']}, Type: {t['dtype']}")

    print("\nTFLite model created and saved successfully!")


if __name__ == '__main__':
    create_and_save_tflite_model()