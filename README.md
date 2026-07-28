# Cardiovascular risk stratification with KNN

A clinical decision support model that sorts heart disease into **five severity levels**, not a binary yes/no, from 920 patient records in the UCI Heart Disease dataset.

The write-up is deliberately as much about where the model fails as where it works.

![Confusion matrix at k = 5](docs/confusion-matrix.png)

## Why KNN, and not a neural network

Chosen on three grounds, not convenience:

**Interpretability.** In a clinical setting a prediction you cannot justify is a prediction nobody acts on. KNN is instance based, so a result can be explained by showing the actual historical patients that produced it. A multilayer perceptron cannot offer that without extra machinery.

**Data efficiency.** 920 instances with under 20 features. That is comfortably inside the regime where distance based methods match deeper models, and well below where a network can be trained without heavy regularisation.

**Native multi-class.** Five ordered classes, handled directly by majority vote, with no one-vs-rest scaffolding.

## Preprocessing

Euclidean distance assumes every dimension is equally scaled and equally meaningful, so the pipeline exists to make that assumption true:

- **Hybrid imputation.** `slope`, `ca` and `thal` have missing values; dropping those rows would have discarded over 30 percent of the data and introduced selection bias. Numeric columns take the mean, categoricals the mode.
- **One-hot encoding** for categoricals, so no false ordinal relationship is implied between, say, chest pain types.
- **Z-score scaling.** The single most important step. Unscaled, serum cholesterol at ~200 mg/dL would swamp ST depression at ~1.5 mm and the smaller feature would be effectively invisible to the distance metric.

## Choosing k

Trained and validated across k = 1 to 40, tracking error rate. The curve is the textbook U shape: volatile at k = 1 from fitting noise, minimised at **k = 5**, then rising again as boundaries oversmooth and distinct disease clusters merge.

## Results, read honestly

Overall accuracy is **57 percent**, which sounds mediocre until you look at where the errors are.

- **Class 0, healthy: recall 0.89.** As a triage filter this is genuinely useful. It correctly clears 89 percent of healthy patients, letting specialists spend time on the flagged remainder. A false positive costs an appointment; a false negative costs far more.
- **Classes 1 to 4, severity: degrades sharply, class 4 recall 0.25.** Caused by class imbalance. In sparse regions the vote is dominated by more numerous neighbouring classes.

The failure mode matters more than the number: class 4 patients are rarely misclassified as *healthy*. They get misplaced into class 2 or 3. The model reliably detects that disease is present and is unreliable about grading how severe it is. That is the right way round for a screening tool, and it is a limitation to state rather than average away.

## What would fix it

SMOTE to synthesise minority class examples and rebalance the decision boundaries; feature selection or PCA to strip noise features that distort the distance metric.

## Running it

```bash
pip install scikit-learn pandas numpy matplotlib seaborn jupyter
jupyter notebook heart_disease_knn.ipynb
```

Dataset: [UCI Heart Disease](https://www.kaggle.com/datasets/redwankarimsony/heart-disease-data), 920 instances.
