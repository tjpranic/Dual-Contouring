using NUnit.Framework;
using UnityEngine;

public class SymmetricMatrix3x3Test : ImplementationTest {

    protected override string shader { get; } = "Tests/SymmetricMatrix3x3Test";

    [Test]
    public void multiplyVectorTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 1.822448f, 1.247129f, 0.7164904f, 0.8687181f, 0.4893599f, 0.3088341f );
        var parameter1 = new Vector3( -0.3886719f, -0.2688802f, -0.155599f );

        var expectedResult = parameter0.multiplyVector( parameter1 );
        var actualResult   = this.testKernel(
            "multiplyVectorTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            Vector3.zero
        );

        Assert.AreEqual( expectedResult.x, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.y, actualResult.y, epsilon );
        Assert.AreEqual( expectedResult.z, actualResult.z, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );
                var parameter1 = this.generateRandomVector3( );

                var expectedResult = parameter0.multiplyVector( parameter1 );
                var actualResult   = this.testKernel(
                    "multiplyVectorTest",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    parameter1,
                    Vector3.zero
                );

                Assert.AreEqual( expectedResult.x, actualResult.x, epsilon );
                Assert.AreEqual( expectedResult.y, actualResult.y, epsilon );
                Assert.AreEqual( expectedResult.z, actualResult.z, epsilon );
            }
        );
    }

    [Test]
    public void frobeniusNormTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );

        var expectedResult = parameter0.frobeniusNorm( );
        var actualResult   = this.testKernel(
            "frobeniusNormTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );

                var expectedResult = parameter0.frobeniusNorm( );
                var actualResult   = this.testKernel(
                    "frobeniusNormTest",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    0.0f
                );

                Assert.AreEqual( expectedResult, actualResult, epsilon );
            }
        );
    }

    [Test]
    public void offTest( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2844447f, 0.06770207f, -0.05499619f, 2.503393f, 0.0f, 0.2121617f );

        var expectedResult = parameter0.off( );
        var actualResult   = this.testKernel(
            "offTest",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            0.0f
        );

        Assert.AreEqual( expectedResult, actualResult, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );

                var expectedResult = parameter0.off( );
                var actualResult   = this.testKernel(
                    "offTest",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    0.0f
                );

                Assert.AreEqual( expectedResult, actualResult, epsilon );
            }
        );
    }

    [Test]
    public void rotate01Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.7640296f, 0.6270213f, 0.6270213f, 1.117985f, 0.935941f, 1.117985f );
        var parameter1 = new Vector2( 0.7973828f, 0.6034737f );

        var expectedResult = parameter0.rotate01( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate01Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );
                var parameter1 = this.generateRandomVector2( );

                var expectedResult = parameter0.rotate01( parameter1.x, parameter1.y );
                var actualResult   = this.testKernel(
                    "rotate01Test",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    parameter1,
                    new SVDQEF.SymmetricMatrix3x3.Data( )
                );

                Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
                Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
                Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
                Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
                Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
                Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
            }
        );
    }

    [Test]
    public void rotate02Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2894885f, 0.0f, -0.06483984f, 1.592526f, 1.124694f, 1.117985f );
        var parameter1 = new Vector2( 0.9969881f, -0.07755417f );

        var expectedResult = parameter0.rotate02( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate02Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );
                var parameter1 = this.generateRandomVector2( );

                var expectedResult = parameter0.rotate02( parameter1.x, parameter1.y );
                var actualResult   = this.testKernel(
                    "rotate02Test",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    parameter1,
                    new SVDQEF.SymmetricMatrix3x3.Data( )
                );

                Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
                Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
                Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
                Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
                Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
                Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
            }
        );
    }

    [Test]
    public void rotate12Test( ) {
        var parameter0 = new SVDQEF.SymmetricMatrix3x3( 0.2844447f, 0.08722472f, 0.0f, 1.592526f, 1.121307f, 1.123029f );
        var parameter1 = new Vector2( 0.7761799f, -0.6305115f );

        var expectedResult = parameter0.rotate12( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate12Test",
            new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.SymmetricMatrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );

        this.repeat(
            100,
            ( ) => {
                var parameter0 = this.generateRandomSymmetricMatrix3x3( );
                var parameter1 = this.generateRandomVector2( );

                var expectedResult = parameter0.rotate12( parameter1.x, parameter1.y );
                var actualResult   = this.testKernel(
                    "rotate12Test",
                    new SVDQEF.SymmetricMatrix3x3.Data( parameter0 ),
                    parameter1,
                    new SVDQEF.SymmetricMatrix3x3.Data( )
                );

                Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
                Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
                Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
                Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
                Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
                Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
            }
        );
    }

}
