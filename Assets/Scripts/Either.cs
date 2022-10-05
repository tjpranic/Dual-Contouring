using System;

public abstract class Either<T0, T1> {

    private Either( ) { }

    public sealed class Type0 : Either<T0, T1> {

        public T0 data { get; }

        public Type0( T0 data ) {
            this.data = data;
        }

        public static implicit operator Type0( T0 data ) {
            return new Type0( data );
        }

    }

    public sealed class Type1 : Either<T0, T1> {

        public T1 data { get; }

        public Type1( T1 data ) {
            this.data = data;
        }

        public static implicit operator Type1( T1 data ) {
            return new Type1( data );
        }

    }

    public TResult visit<TResult>( Func<T0, TResult> visitor0, Func<T1, TResult> visitor1 ) {
        return this switch {
            Type0 type0 => visitor0( type0.data ),
            Type1 type1 => visitor1( type1.data ),
            _ => throw new Exception( "Unknown type specified." ),
        };
    }

    public void visit( Action<T0> visitor0, Action<T1> visitor1 ) {
        switch( this ) {
            case Type0 type0:
                visitor0( type0.data );
                break;
            case Type1 type1:
                visitor1( type1.data );
                break;
            default:
                throw new Exception( "Unknown type specified." );
        };
    }

}