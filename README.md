# UERJ Sats - Repositório de Códigos Históricos (2023 - 2024)

![Status](https://img.shields.io/badge/Status-Arquivo%20%2F%20Hist%C3%B3rico-lightgrey)
![Instituição](https://img.shields.io/badge/UERJ-Sats-blue)
![Mantenedor](https://img.shields.io/badge/Mantenedor-katarynecunha28-purple)

Este repositório centraliza, organiza e preserva o histórico de códigos de voo, firmwares, algoritmos de sensores e ferramentas legadas desenvolvidos para as missões da **UERJ Sats** durante os ciclos de **2023** a **2025**.

O objetivo deste espaço é servir como uma base de consulta técnica e preservação do conhecimento das primeiras missões para as novas gerações de membros da equipe.

---

## 🛰️ Estrutura das Missões e Pastas

Com base no histórico real de desenvolvimento visível na raiz do repositório (`image_a0ccdd.png`), as pastas estão organizadas da seguinte forma:

### 🚀 Missões e Competições
*   **`Missão Kurumim`:** Códigos e rotinas desenvolvidos para a competição em que a equipe participou, onde o satélite Bertola-1 alcançou a impressionante marca de **26 km de altitude**. Contém algoritmos de leitura de dados de voo e gerenciamento de memória (como a leitura de EEPROM).
*   **`LASC2023`:** Firmwares e códigos finais desenvolvidos e embarcados para a competição internacional *Latin American Space Challenge* (LASC) no ciclo de 2023[cite: 3].
*   **`2025/PIQUENIQUE`:** Códigos focados nos testes de comunicação e validação de alcance das **antenas com rádio LoRa**, além de testes de integração com displays.
*   **`CubeDesign2024`:** Scripts e lógicas desenvolvidos para o ecossistema de satélites voltados para a competição CubeDesign do INPE[cite: 3].
*   **`Missão Guacamole`:** Códigos dedicados ao planejamento e integração de subsistemas para uma missão de lançamento de balão atmosférico (lançamento que, por motivos operacionais, acabou não acontecendo).

### 🛠️ Hardware, Bibliotecas e Bancada
*   **`Bibliotecas PCB`:** Arquivos de suporte, footprints e componentes criados e utilizados no software **Autodesk Eagle** para o design e manufatura das placas de circuito impresso (PCBs) da equipe.
*   **`Bibliotecas GPS`:** Coletânea de bibliotecas externas e parsers de geolocalização estruturados especificamente para o ambiente de desenvolvimento da **Arduino IDE**.
*   **`Códigos de Teste`:** Scripts rápidos e isolados feitos para testar o funcionamento de componentes eletrônicos específicos em bancada antes da integração final.
  
---

## 📝 Como Reaproveitar este Material

Se você é um novo membro técnico da **UERJ Sats**, utilize este espaço para:
*   Estudar a lógica de transmissão e alcance de rádio usando a tecnologia LoRa (pasta `2025/PIQUENIQUE`).
*   Entender a arquitetura de um sistema de software que suportou um voo real de alta altitude a 26 km (pasta `Missão Kurumim`).

---
*Orgulho de manter viva a história e o legado da UERJ Sats!* 🥑🛰️🔥
