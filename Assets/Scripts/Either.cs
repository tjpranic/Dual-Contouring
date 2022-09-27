using System;

public abstract class Either<T1, T2> {

    private Either( ) { }

    public sealed class Type1 : Either<T1, T2> {

        public T1 data { get; }

        public Type1( T1 data ) {
            this.data = data;
        }

        public static implicit operator Type1( T1 data ) {
            return new Type1( data );
        }

    }

    public sealed class Type2 : Either<T1, T2> {

        public T2 data { get; }

        public Type2( T2 data ) {
            this.data = data;
        }

        public static implicit operator Type2( T2 data ) {
            return new Type2( data );
        }

    }

    public TResult visit<TResult>(
        Func<T1, TResult> visitor1,
        Func<T2, TResult> visitor2
    ) {
        return this switch {
            Type1 type1 => visitor1( type1.data ),
            Type2 type2 => visitor2( type2.data ),
            _ => throw new Exception( "Unknown type specified." ),
        };
    }

    public void visit(
        Action<T1> visitor1,
        Action<T2> visitor2
    ) {
        switch( this ) {
            case Type1 type1:
                visitor1( type1.data );
                break;

            case Type2 type2:
                visitor2( type2.data );
                break;

            default:
                throw new Exception( "Unknown type specified." );
        };
    }

}