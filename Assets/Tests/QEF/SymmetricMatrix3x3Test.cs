using NUnit.Framework;
using UnityEngine;

public class SymmetricMatrix3x3Test : ImplementationTest {

    protected override string shader { get; } = "Tests/SymmetricMatrix3x3Test";

    [Test]
    public void multiplyVectorTest( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 1.822448f, 1.247129f, 0.7164904f, 0.8687181f, 0.4893599f, 0.3088341f );
        var vector             = new Vector3( -0.3886719f, -0.2688802f, -0.155599f );

        var expectedResult = symmetricMatrix3x3.multiplyVector( vector );
        var actualResult   = this.testKernel(
            "multiplyVectorTest",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            vector,
            Vector3.zero
        );

        Assert.AreEqual( expectedResult.x, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.y, actualResult.y, epsilon );
        Assert.AreEqual( expectedResult.z, actualResult.z, epsilon );
    }

    [Test]
    public void frobeniusNormTest( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );

        var expectedResult = symmetricMatrix3x3.frobeniusNorm( );
        var actualResult   = this.testKernel(
            "frobeniusNormTest",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );
    }

    [Test]
    public void offTest( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 0.2844447f, 0.06770207f, -0.05499619f, 2.503393f, 0.0f, 0.2121617f );

        var expectedResult = symmetricMatrix3x3.off( );
        var actualResult   = this.testKernel(
            "offTest",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );
    }

    [Test]
    public void rotate01Test( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var cs                 = new Vector2( 0.7973828f, 0.6034737f );

        var expectedResult = symmetricMatrix3x3.rotate01( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate01Test",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            cs,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate02Test( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 0.2894885f, 0.0f, -0.06483984f, 1.592526f, 1.124694f, 1.117985f );
        var cs                 = new Vector2( 0.9969881f, -0.07755417f );

        var expectedResult = symmetricMatrix3x3.rotate02( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate02Test",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            cs,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate12Test( ) {
        var symmetricMatrix3x3 = new SVDQEF.SymmetricMatrix3x3( 0.2844447f, 0.08722472f, 0.0f, 1.592526f, 1.121307f, 1.123029f );
        var cs                 = new Vector2( 0.7761799f, -0.6305115f );

        var expectedResult = symmetricMatrix3x3.rotate12( cs.x, cs.y );
        var actualResult   = this.testKernel(
            "rotate12Test",
            new SVDQEF.SymmetricMatrix3x3.Data( symmetricMatrix3x3 ),
            cs,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

}
