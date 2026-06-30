# Projeto-bixo
Repositório oficial do Grupo 3 no Projeto Bixo 2026. Desenvolvimento de um robô autônomo com ESP32-C3, Ponte H L298N, odometria e integração com Micro-ROS.

---

## 👥 Autores (Membros do Grupo 3)

* **[João Neto]**
* **[Miguel Novais]**
* **[Thiago Perini]**
* **[Leonardo Ferreira]**
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