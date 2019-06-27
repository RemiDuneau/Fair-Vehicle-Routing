import java.util.Objects;

public class Tuple<X,Y> {
    public final X x;
    public final Y y;

    public Tuple(X x, Y y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public boolean equals(Object other) {
        if (other == this) return true;
        if (!(other instanceof Tuple)) return false;

        Tuple<X,Y> otherTuple = (Tuple<X,Y>) other;
        if (x.equals(otherTuple.x) && y.equals(otherTuple.y)) return true;
        else return false;
    }
}
