# Calculation sheet
For developing test cases, I need somewhere to record my math that is not in a messy docstring. makes that code super messy and basically unreadable.

## `test_localize`
### covariance calculation
making sure `np.cov()` does what I want it to.

original particles:
```
[0.0, 1.0, 10.0]
[1.0, 2.0, 20.0]
[2.0, 3.0, 30.0]
```

first the particles are put into columns like `[x, y, heading]`. 
```
             x       y       heading
particle 1   0       1       10
particle 2   1       2       20
particle 3   2       3       30
```

#### Definitions
Covariance: _How two different variables change together._
$$Cov(X,Y) = \frac{\sum(X_i -\={X})(Y_i -\={Y})}{n - 1}$$

Variance: _How one variable spreads out from its own mean._
$$Var(X) = \frac{\sum(X_i -\={X})^2}{n - 1}$$

> Where $\={X}$ is the mean and $(X_i - \={X})$ is the deviance. $n$ = number of variables

#### Steps to calculate without `np.cov()`
1. Find the mean:
$$ \={X} = \frac{0 + 1 + 2}{3} = 1\\[8mu]
\={Y} = \frac{1+2+3}{3} = 2\\[8mu]
\={\Theta} = \frac{10+20+30}{3} = 20$$

2. Calculate deviations from the mean

| row | $(X_i-\={X})$ | $(Y_i-\={Y})$ | $(\Theta_i-\={\Theta})$ |
|-----|---------------|---------------|-------------------------|
|  1  |    $0-1=-1$   |    $1-2=-1$   |       $10-20=-10$       |
|  2  |    $1-1= 0$   |    $2-2= 0$   |       $20-20=  0$       |
|  3  |    $2-1= 1$   |    $3-2= 1$   |       $30-20= 10$       |

3. Compute matrix elements
> $n = 3, \thickspace n-1 = 2$

Diagonals: </br>
$Var(X) = \frac{(-1)^2 + (0)^2 + (1)^2}{2} = \frac{2}{2} = 1\\[8mu]$
$Var(Y) = \frac{(-1)^2 + (0)^2 + (1)^2}{2} = \frac{2}{2} = 1\\[8mu]$
$Var(\Theta) = \frac{(-10)^2 + (0)^2 + (10)^2}{2} = \frac{100}{2} = 100\\[8mu]$

$Cov(X,Y) = \frac{(-1\times-1) + (0\times 0) + (1\times 1)}{2} = \frac{2}{2} = 1\\[8mu]$
$Cov(X, \Theta) = \frac{(-1\times-10) + (0\times 0) + (1\times 10)}{2} = \frac{20}{2} = 10\\[8mu]$
$Cov(\Theta, Y) = \frac{(-10\times-1) + (0\times 0) + (10\times 1)}{2} = \frac{20}{2} = 10\\[8mu]$

How the matrix is shaped:
```
              x       y        heading
x             Var(x)  Cov(x,y) Cov(x,heading)
y             Cov(y,x) Var(y)  Cov(y,heading)
heading       ...     ...      Var(heading)
```

end result:
```
[1.0, 1.0, 10.0]
[1.0, 1.0, 10.0]
[10.0, 10.0, 100.0]
```

<!-- Σ=1n−1(DT×D) | Σ=1𝑛−1(𝐷𝑇×𝐷) -->
> shortcut way is to use the one-step matrix method: $\Sigma = \frac{1}{n-1}(D^T \times D)$ 

### motion update sucessful--> returns estimated pose
instead of calling the actual motion model, use a static simple update.

original particles:
```
[0.0, 0.0, 0.0]
[2.0, 2.0, 10.0]
```
so update function gets:
```
Odometry: dx=1, dy=2, dθ=3
Sensor reading: empty list
```
> the particles are equally weighted `(0.5, 0.5)`

then add the delta to the original particles to get the moved ones.

`original particle + delta = moved particle`

$[0, 0, 0] + [1, 2, 3] = [1, 2, 3]$ </br>
$[2, 2, 10] + [1, 2, 3] = [3, 4, 13]$ </br>

$x = (1 + 3) / 2 = 2$ </br>
$y = (2 + 4) / 2 = 3$ </br>
$th = (3 + 13) / 2 = 8$ </br>

So the expected pose should be: `(2.0, 3.0, 8.0)`
