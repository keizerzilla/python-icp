# python-icp

Implementação do algoritmo ICP (Iterative Closest Point) para nuvens de pontos 3D usando Python.

## Dependências

Para instalar as dependências necessárias, execute o gerenciador de pacotes ```pip3``` na raíz do repositório:

```
pip3 install -r requirements.txt
```

## Código

Todas as funções auxiliares (carregar nuvem, encontrar transformação, vizinhos mais próximo...) bem como a função principal que executa o ICP estão no arquivo ```registration.py```. Cada função foi documentada usando o padrão de documentação de funções do Numpy.

## Usando

O script ```test.py``` traz um exemplo onde o algoritmo registra as nuvens do coelho 0º como origem e o colheo 45º como destino. Para mais testes, a pasta ```clouds``` foi adicionada com nuvens bastante usadas na literatura. Uma sugestão é criar um programa que recebe argumentos da linha de comando tais como nuvem de origem, nuvem de destino, número de iterações e erro e retorna uma visualização, as transformação final, etc.