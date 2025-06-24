package chipyard

import chisel3._

import org.chipsalliance.cde.config.{Config}

// ---------------------
//serv Configs
// ---------------------

// Multi-core and 32b heterogeneous configs are supported

class ServConfig extends Config(
  new serv.WithNServCores(1) ++
  new chipyard.config.WithInclusiveCacheWriteBytes(4) ++
  new chipyard.config.AbstractConfig)
