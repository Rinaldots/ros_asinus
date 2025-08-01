# Otimização de Tópicos RTABMap - Asinus SLAM

## Problema Identificado

O RTABMap estava se inscrevendo em muitos tópicos desnecessários, incluindo:

- `/apriltag/detections`
- `/aruco/detections`
- `/aruco_opencv/detections`
- `/global_pose`
- `/goal`
- `/goal_node`
- `/gps/fix`
- `/imu`
- `/initialpose`
- `/landmark_detection`
- `/landmark_detections`
- `/tag_detections`
- `/user_data_async`

## Solução Implementada

### 1. Configuração Otimizada (`rtabmap_optimized.yaml`)

Criamos um arquivo de configuração específico que desabilita explicitamente todos os sensores não utilizados:

```yaml
# Subscription settings - Only enable sensors we actually have
subscribe_rgbd: true          # ✅ Temos câmera RGBD
subscribe_scan: true          # ✅ Temos LIDAR
subscribe_stereo: false       # ❌ Não temos
subscribe_rgb: false          # ❌ Usamos RGBD em vez de RGB separado
subscribe_depth: false        # ❌ Usamos RGBD em vez de depth separado
subscribe_odom_info: false    # ❌ Não precisamos
subscribe_user_data: false    # ❌ Não usamos
subscribe_aruco: false        # ❌ Não temos detecção ArUco
subscribe_landmarks: false    # ❌ Não temos landmarks
subscribe_target: false       # ❌ Não temos targets
subscribe_apriltag: false     # ❌ Não temos AprilTags
subscribe_imu: false          # ❌ IMU é processado pelo EKF
subscribe_gps: false          # ❌ GPS é processado pelo EKF
```

### 2. Sensores Utilizados

O robô Asinus utiliza apenas:

- **Câmera RGBD**: Para odometria visual e mapeamento
- **LIDAR**: Para detecção de obstáculos e mapeamento 2D
- **Odometria filtrada**: Vinda do EKF que funde IMU + GPS + odometria das rodas

### 3. Benefícios da Otimização

- **Menor uso de CPU**: Redução significativa no processamento
- **Menor uso de memória**: Menos buffers e filas desnecessários
- **Rede mais limpa**: Menos tópicos sendo monitorados
- **Inicialização mais rápida**: Menos dependências para aguardar
- **Logs mais limpos**: Menos mensagens de erro sobre tópicos não encontrados

### 4. Parâmetros Mantidos

Os parâmetros de SLAM foram mantidos otimizados para navegação em ambientes externos:

- **Reg/Strategy**: "1" (ICP + Visual)
- **Reg/Force3DoF**: "true" (SLAM 2D)
- **Grid/FromDepth**: "false" (Usar LIDAR para grid)
- **RGBD/LocalRadius**: "5" (Detecção de loop closure local)

### 5. Configuração de Sincronização

- **approx_sync**: true (sincronização aproximada)
- **sync_queue_size**: 10 (tamanho adequado da fila)
- **wait_for_transform**: 0.2s (tempo limite para transformações)

## Verificação

Para verificar se a otimização funcionou, execute:

```bash
# Listar tópicos ativos do RTABMap
ros2 topic list | grep rtabmap

# Verificar inscrições de um nó específico
ros2 node info /rtabmap_slam
```

Agora o RTABMap deve se inscrever apenas nos tópicos necessários:
- `/rgbd_camera/image`
- `/rgbd_camera/depth_image`
- `/rgbd_camera/camera_info`
- `/scan`
- `/odometry/filtered`

## Arquivos Modificados

1. **`slam.launch.py`**: Atualizado para usar configuração YAML externa
2. **`rtabmap_optimized.yaml`**: Nova configuração otimizada
3. **Remappings**: Mantidos para mapear corretamente os tópicos do robô
