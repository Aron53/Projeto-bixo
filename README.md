# Projeto-bixo
Repositório oficial do Grupo 3 no Projeto Bixo 2026. Desenvolvimento de um robô autônomo com ESP32-C3, Ponte H L298N, odometria e integração com Micro-ROS.

---

## 👥 Autores (Membros do Grupo 3)

* **[João Neto]**
* **[Tiago Perini]**
* **[Mateus]**

---

## 📐 Dimensões do Robô (Unidades em mm)

Com base no desenho técnico bidimensional gerado na aba de *Drawing*, a estrutura final do conjunto montado possui as seguintes dimensões limite:

* **Largura Total (ponta a ponta das rodas):** 257.40 mm
* **Comprimento Total do Chassi:** 250.00 mm
* **Altura Estrutural (até a placa superior):** 93.70 mm
* **Altura Total Máxima (até o topo da varinha):** 160.75 mm

---

## 📸 Visualização do Projeto

![Desenho Técnico do Robô](chassi_medidas/Drawing%201_novo.png)

---

## 🛠️ Escolhas Mecânicas da Equipe

Durante o processo de design e montagem no Onshape, nossa equipe tomou decisões de engenharia estratégicas voltadas para a estabilidade, facilidade de manufatura e manutenção:

1. **Geometria Octogonal:** Escolhida para eliminar cantos vivos de 90°, facilitando manobras e evitando colisões bruscas nas quinas do chassi em ambientes estreitos.
2. **Configuração de Tração Dinâmica:** Optamos por posicionar duas rodas motrizes nas laterais traseiras acopladas diretamente aos motores DC com caixa de redução, combinadas a uma roda na região frontal. Isso garante alto índice de agilidade e raio de giro reduzido.
3. **Distribuição de Componentes e Centro de Massa:** A eletrônica pesada (baterias 18650 e Ponte H L298N) foi fixada estrategicamente mais próxima ao solo, garantindo estabilidade contra tombamentos.
4. **Customização Estética:** Foi adicionado um suporte customizado na chapa superior para o encaixe de uma varinha decorativa na ponta, agregando identidade visual ao projeto sem interferir na dinâmica estrutural.

---

## 📁 Organização Estrutural do Repositório

Os arquivos foram separados dentro do repositório de acordo com as diretrizes de manufatura:

* **`chassi_medidas/`**
  * `Drawing 1.pdf` -> Desenho técnico 2D completo em formato PDF.
  * `Drawing 1.png` -> Imagem do desenho técnico exibida neste README.
  * `modelo_completo_3d.stl` -> Arquivo tridimensional de visualização global do robô montado.
* **`componentes/`**
  * `base_inferior.stl` -> Chapa base inferior do chassi para impressão 3D (unidade em mm).
  * `base_superior.stl` -> Chapa base superior do chassi para impressão 3D (unidade em mm).
  * `espacador.stl` -> Pilares de união e distanciamento entre as chapas.
  * `suporte_motor.stl` -> Presilhas/suportes estruturais para os motores DC amarelos.
  * `varinha_estetica.stl` -> Elemento de personalização do topo.

## Circuito
![Esquema do circuito](circuito\sketch_circuito_bb.png)

---

## 🧭 Odometria

A odometria estima **onde o robô está** (`x`, `y` e orientação `θ`) a partir do giro das rodas. Cada motor tem um **encoder** que gera pulsos ao girar; contando esses pulsos, calculamos quanto cada roda andou e como o robô se moveu.

### ⚙️ Como funciona

1. **Contagem de pulsos:** uma interrupção dispara a cada pulso do canal A do encoder. O canal B indica o sentido: `HIGH` soma (`ticks++`) e `LOW` subtrai (`ticks--`).
2. **Leitura a cada 50 ms (20 Hz):** o `loop()` usa `millis()`, sem `delay()`. Os contadores são copiados com as interrupções desligadas por um instante, para não corromper os dados.
3. **Cálculo:** a diferença de pulsos desde a última leitura vira distância de cada roda, deslocamento e rotação do robô, e a posição é atualizada (fórmulas abaixo).

### 📏 Parâmetros

| Parâmetro | Símbolo | Variável no código | Valor |
|---|---|---|---|
| Raio da roda | R | `RAIO_RODA` | 0,0336 m |
| Distância entre as rodas | L | `DIST_RODAS` | 0,2140 m |
| Pulsos por volta da roda | N | `PPR_ENCODER` | 330 |
| Tempo de amostragem | Δt | `DT` | 0,05 s |

### 🧮 Fórmulas

Os índices $R$ e $L$ indicam a roda direita e a esquerda.

| O que calcula | Fórmula | Em palavras |
|---|---|---|
| Distância de cada roda | $d = \dfrac{ticks}{N}\cdot 2\pi R$ | fração de volta dada × perímetro da roda |
| Deslocamento do robô | $\Delta s = \dfrac{d_R + d_L}{2}$ | média do que as duas rodas andaram |
| Variação do ângulo | $\Delta\theta = \dfrac{d_R - d_L}{L}$ | a diferença entre as rodas faz o robô girar |
| Posição | $\Delta x = \Delta s\cos\theta_k \quad \Delta y = \Delta s\sin\theta_k$ | decompõe o deslocamento em `x` e `y` |
| Velocidades | $v = \dfrac{\Delta s}{\Delta t} \quad \omega = \dfrac{\Delta\theta}{\Delta t}$ | divide pelo tempo do ciclo (usadas no PID) |

A pose é acumulada a cada ciclo: $x_{k+1} = x_k + \Delta x$, $y_{k+1} = y_k + \Delta y$ e $\theta_{k+1} = \theta_k + \Delta\theta$ (no código, `pos_x`, `pos_y` e `pos_theta`).

