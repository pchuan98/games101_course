
## 2D

手动推到线性代数的公式

1. Scale
2. Reflection
3. Shear
4. Rotate
5. move

## Homogeneous Coordinates

add one more dimension to the vector or point

- 2D point $(x,y)$ -> $(x,y,1)$
- 2D vector $(x,y)$ -> $(x,y,0)$

其次，引用一个新的点后，通过第三个维度的值来判断是点还是向量，其中的计算公式如下：

- vector + vector = vector
- point + vector = point
- point + point = undefined -> $\left(\begin{array}{c}x\\y\\w\end{array}\right)$ = $\left(\begin{array}{c}x/w\\y/w\\1\end{array}\right)$
- point - point = vector

例如，point + vector的实质为点在向量移动后，依旧是一个点。

$$
\left[
    \begin{array}{c}
      x^\prime\\y^\prime
    \end{array}
\right]
=
\left[
    \begin{array}{cc}
      a & b\\
      c & d
    \end{array}
\right]
\cdot
\left[
    \begin{array}{c}
      x\\y
    \end{array}
\right]
+
\left[
    \begin{array}{c}
      t_x\\t_y
    \end{array}
\right]
\implies
\left[
    \begin{array}{c}
      x^\prime\\y^\prime\\1
    \end{array}
\right]
=
\left[
    \begin{array}{ccc}
      a & b & t_x\\
      c & d & t_y\\
      0 & 0 & 1
    \end{array}
\right]
\cdot
\left[
    \begin{array}{c}
      x\\y\\1
    \end{array}
\right]
$$

## 3D

- point $(x,y,z,1)$
- vector $(x,y,z,0)$

## Notes

矩阵变换运算： $A^\prime = M \cdot A$。其中M是变换矩阵，$A$是原始矩阵，$A^\prime$是变换后的矩阵

Rotate默认为逆时针旋转

默认中，二维形式如下:
$$
\left[
    \begin{array}{c}
      x^\prime\\y^\prime
    \end{array}
\right]
=
\left[
    \begin{array}{cc}
      a & b\\
      c & d
    \end{array}
\right]
\cdot
\left[
    \begin{array}{c}
      x\\y
    \end{array}
\right]
$$
