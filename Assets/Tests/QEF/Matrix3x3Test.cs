using NUnit.Framework;
using UnityEngine;

public class Matrix3x3Test : ImplementationTest {

    protected override string shader { get; } = "Tests/Matrix3x3Test";

    [Test]
    public void multiplyVectorTest( ) {
        var parameter0 = new SVDQEF.Matrix3x3( 0.7640296f, 0.0f, 0.0f, 0.0f, 1.117985f, 0.0f, 0.0f, 0.0f, 1.117985f );
        var parameter1 = new Vector3( -0.02622265f, -0.02307659f, -0.02307659f );

        var expectedResult = parameter0.multiplyVector( parameter1 );
        var actualResult   = this.testKernel(
            "multiplyVectorTest",
            new SVDQEF.Matrix3x3.Data( parameter0 ),
            parameter1,
            Vector3.zero
        );

        Assert.AreEqual( expectedResult.x, actualResult.x, epsilon );
        Assert.AreEqual( expectedResult.y, actualResult.y, epsilon );
        Assert.AreEqual( expectedResult.z, actualResult.z, epsilon );
    }

    [Test]
    public void rotate01PostTest( ) {
        var parameter0 = SVDQEF.Matrix3x3.identity;
        var parameter1 = Vector2.zero;

        var expectedResult = parameter0.rotate01Post( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate01PostTest",
            new SVDQEF.Matrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate02PostTest( ) {
        var parameter0 = SVDQEF.Matrix3x3.identity;
        var parameter1 = Vector2.zero;

        var expectedResult = parameter0.rotate02Post( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate02PostTest",
            new SVDQEF.Matrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

    [Test]
    public void rotate12PostTest( ) {
        var parameter0 = SVDQEF.Matrix3x3.identity;
        var parameter1 = Vector2.zero;

        var expectedResult = parameter0.rotate12Post( parameter1.x, parameter1.y );
        var actualResult   = this.testKernel(
            "rotate12PostTest",
            new SVDQEF.Matrix3x3.Data( parameter0 ),
            parameter1,
            new SVDQEF.Matrix3x3.Data( )
        );

        Assert.AreEqual( expectedResult.m00, actualResult.m00, epsilon );
        Assert.AreEqual( expectedResult.m01, actualResult.m01, epsilon );
        Assert.AreEqual( expectedResult.m02, actualResult.m02, epsilon );
        Assert.AreEqual( expectedResult.m10, actualResult.m10, epsilon );
        Assert.AreEqual( expectedResult.m11, actualResult.m11, epsilon );
        Assert.AreEqual( expectedResult.m12, actualResult.m12, epsilon );
        Assert.AreEqual( expectedResult.m20, actualResult.m20, epsilon );
        Assert.AreEqual( expectedResult.m21, actualResult.m21, epsilon );
        Assert.AreEqual( expectedResult.m22, actualResult.m22, epsilon );
    }

}
