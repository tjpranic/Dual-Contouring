using System;

// TODO: fill out the rest of the table to allow symmetrical contouring

public class OctreeContouringTables<Voxel> where Voxel : SurfaceExtractor.Voxel {

    public enum Axis {
        X,
        Y,
        Z
    }

    public enum Position {
        Root                        = -1,
        NegativeXNegativeYNegativeZ =  0,
        PositiveXNegativeYNegativeZ =  1,
        PositiveXNegativeYPositiveZ =  2,
        NegativeXNegativeYPositiveZ =  3,
        NegativeXPositiveYPositiveZ =  4,
        NegativeXPositiveYNegativeZ =  5,
        PositiveXPositiveYNegativeZ =  6,
        PositiveXPositiveYPositiveZ =  7
    }

    public static Octree<Voxel>[][] lookupEdgeNodesWithinNode( Octree<Voxel> node, Axis axis, Position position ) {
        return position switch {
            Position.Root or // just handle the root node as though it was the first child voxel, it doesn't really matter tbh
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[3], node.children[5], node.children[4] },
                    new Octree<Voxel>[] { node.children[1], node.children[2], node.children[6], node.children[7] }
                },
                Axis.Y => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[1], node.children[3], node.children[2] },
                    new Octree<Voxel>[] { node.children[5], node.children[6], node.children[4], node.children[7] }
                },
                Axis.Z => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[5], node.children[1], node.children[6] },
                    new Octree<Voxel>[] { node.children[3], node.children[4], node.children[2], node.children[7] }
                },
                _ => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

    public static Octree<Voxel>[][] lookupFacePairsWithinNode( Octree<Voxel> node, Axis axis, Position position ) {
        return position switch {
            Position.Root or
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[1] },
                    new Octree<Voxel>[] { node.children[3], node.children[2] },
                    new Octree<Voxel>[] { node.children[5], node.children[6] },
                    new Octree<Voxel>[] { node.children[4], node.children[7] }
                },
                Axis.Y => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[5] },
                    new Octree<Voxel>[] { node.children[1], node.children[6] },
                    new Octree<Voxel>[] { node.children[3], node.children[4] },
                    new Octree<Voxel>[] { node.children[2], node.children[7] }
                },
                Axis.Z => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] { node.children[0], node.children[3] },
                    new Octree<Voxel>[] { node.children[1], node.children[2] },
                    new Octree<Voxel>[] { node.children[5], node.children[4] },
                    new Octree<Voxel>[] { node.children[6], node.children[7] }
                },
                _ => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

    public static Octree<Voxel>[][] lookupFacePairsWithinFacePairs( Octree<Voxel>[] nodes, Axis axis, Position position ) {
        return position switch {
            Position.Root or
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[4]
                    }
                },
                Axis.Y => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[2]
                    }
                },
                Axis.Z => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[6]
                    },
                },
                _ => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

    public static ( Octree<Voxel>[], Axis )[] lookupEdgeNodesWithinFacePairs( Octree<Voxel>[] nodes, Axis axis, Position position ) {
        return position switch {
            Position.Root or
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => new ( Octree<Voxel>[], Axis )[] {
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                        },
                        Axis.Y
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[4]
                        },
                        Axis.Y
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[1],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                        },
                        Axis.Z
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[4]
                        },
                        Axis.Z
                    ),
                },
                Axis.Y => new ( Octree<Voxel>[], Axis )[] {
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3]
                        },
                        Axis.X
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[2]
                        },
                        Axis.X
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[5],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                        },
                        Axis.Z
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[2]
                        },
                        Axis.Z
                    ),
                },
                Axis.Z => new ( Octree<Voxel>[], Axis )[] {
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5]
                        },
                        Axis.X
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[6]
                        },
                        Axis.X
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[3],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[0],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1]
                        },
                        Axis.Y
                    ),
                    (
                        new Octree<Voxel>[] {
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                            nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                            nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[6]
                        },
                        Axis.Y
                    ),
                },
                _ => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

    public static Octree<Voxel>[][] lookupEdgeNodesWithinEdgeNodes( Octree<Voxel>[] nodes, Axis axis, Position position ) {
        return position switch {
            Position.Root or
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[4],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[5],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[3],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[6],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[2],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[1]
                    }
                },
                Axis.Y => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[2],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[3],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[1],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[4],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[6],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[5]
                    },
                },
                Axis.Z => new Octree<Voxel>[][] {
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[6],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[1],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[5],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[0]
                    },
                    new Octree<Voxel>[] {
                        nodes[0].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[0] : nodes[0].children[7],
                        nodes[1].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[1] : nodes[1].children[2],
                        nodes[2].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[2] : nodes[2].children[4],
                        nodes[3].data.type != SurfaceExtractor.Voxel.Type.Internal ? nodes[3] : nodes[3].children[3]
                    },
                },
                _ => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                _ => throw new NotImplementedException( ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

    public static SurfaceExtractor.Edge lookupEdgeWithinEdgeNodes( Octree<Voxel>[] nodes, Axis axis, Position position ) {
        return position switch {
            Position.Root or
            Position.NegativeXNegativeYNegativeZ => axis switch {
                Axis.X => nodes[0].data.edges[3],
                Axis.Y => nodes[0].data.edges[7],
                Axis.Z => nodes[0].data.edges[11],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYNegativeZ => axis switch {
                Axis.X => nodes[0].data.edges[3],
                Axis.Y => nodes[0].data.edges[6],
                Axis.Z => nodes[0].data.edges[10],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXNegativeYPositiveZ => axis switch {
                Axis.X => nodes[0].data.edges[2],
                Axis.Y => nodes[0].data.edges[4],
                Axis.Z => nodes[0].data.edges[10],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.NegativeXNegativeYPositiveZ => axis switch {
                Axis.X => nodes[0].data.edges[2],
                Axis.Y => nodes[0].data.edges[5],
                Axis.Z => nodes[0].data.edges[11],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.NegativeXPositiveYPositiveZ => axis switch {
                Axis.X => nodes[0].data.edges[0],
                Axis.Y => nodes[0].data.edges[5],
                Axis.Z => nodes[0].data.edges[9],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.NegativeXPositiveYNegativeZ => axis switch {
                Axis.X => nodes[0].data.edges[1],
                Axis.Y => nodes[0].data.edges[7],
                Axis.Z => nodes[0].data.edges[9],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXPositiveYNegativeZ => axis switch {
                Axis.X => nodes[0].data.edges[1],
                Axis.Y => nodes[0].data.edges[6],
                Axis.Z => nodes[0].data.edges[8],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            Position.PositiveXPositiveYPositiveZ => axis switch {
                Axis.X => nodes[0].data.edges[0],
                Axis.Y => nodes[0].data.edges[4],
                Axis.Z => nodes[0].data.edges[8],
                _      => throw new Exception( "Unknown axis specified" ),
            },
            _ => throw new Exception( "Unknown position specified" ),
        };
    }

}

